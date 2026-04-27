# Copyright 2026 RealSense, Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Verify an rs-converted .db3 round-trips cleanly through `ros2 bag play`:
# every sensor_msgs/msg/Image message delivered by ros2 bag play must be
# byte-identical to the corresponding frame in the source .bag.

import hashlib, os, shutil, subprocess, sys, time
import pytest, rclpy
from rclpy.qos import HistoryPolicy, QoSProfile
from sensor_msgs.msg import Image

sys.path.append(os.path.abspath(os.path.dirname(__file__) + "/../utils"))
sys.path.append(os.path.abspath(os.path.dirname(__file__) + "/../../scripts"))
from pytest_rs_utils import get_rosbag_file_path
from importRosbag.importRosbag import importRosbag


TOPICS = [
    "/device_0/sensor_1/Color_0/image/data",
    "/device_0/sensor_0/Depth_0/image/data",
    "/device_0/sensor_0/Infrared_1/image/data",
]


def get_frame_hash(frames):
    return [hashlib.sha256(f).hexdigest() for f in frames]


@pytest.mark.rosbag
def test_ros2_bag_play_db3(tmp_path):
    rs_convert = shutil.which("rs-convert")
    assert rs_convert, "rs-convert not on PATH (was librealsense built/installed?)"
    # rosdistro's librealsense2 may predate the bag-to-db3 converter (added
    # in librealsense PR #14882). pre-release.yml's own header notes this:
    # "may fail due to outdated librealsense2 on the ROS package servers".
    rs_help = subprocess.run([rs_convert, "--help"], capture_output=True).stdout.decode()
    if "output-db3" not in rs_help:
        pytest.skip(f"{rs_convert} predates --output-db3; rebuild librealsense from source")
    assert subprocess.run(["ros2", "bag", "play", "--help"],
                          capture_output=True).returncode == 0, \
        "ros2 bag play not available (install ros-${ROS_DISTRO}-ros2bag)"
    bag = get_rosbag_file_path("outdoors_1color.bag")
    db3 = str(tmp_path / "out.db3")
    subprocess.run([rs_convert, "-i", bag, "-D", db3],
                   check=True, capture_output=True, timeout=120)

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("db3_play_subscriber")
    # ros2 bag play publishes RELIABLE by default (no metadata.yaml beside the
    # .db3); KEEP_ALL + large depth prevents subscriber-side drops. Stream-hash
    # in the callback so raw frame buffers don't pile up in memory.
    qos = QoSProfile(history=HistoryPolicy.KEEP_ALL, depth=1000)
    received = {t: [] for t in TOPICS}
    for t in TOPICS:
        node.create_subscription(
            Image, t,
            lambda m, t=t: received[t].append(hashlib.sha256(bytes(m.data)).hexdigest()),
            qos)

    proc = subprocess.Popen(["ros2", "bag", "play", db3],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        deadline = time.time() + 60
        while proc.poll() is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        drain = time.time() + 2
        while time.time() < drain:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        out = proc.stdout.read().decode(errors="replace") if proc.stdout else ""
        node.destroy_node()
        rclpy.shutdown()

    assert proc.returncode == 0, \
        f"ros2 bag play failed (rc={proc.returncode}):\n{out}"

    src = importRosbag(bag, importTopics=TOPICS, log='ERROR', disable_bar=True)
    for t in TOPICS:
        src_h = get_frame_hash(f.tobytes() for f in src[t]['frames'])
        rx_h = received[t]
        assert rx_h and (rx_h == src_h[:len(rx_h)] or rx_h == src_h[-len(rx_h):]), \
            f"{t}: content mismatch (got {len(rx_h)} / {len(src_h)} source frames)"
