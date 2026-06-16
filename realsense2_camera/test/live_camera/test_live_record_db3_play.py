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

# Live-camera variant of test_rosbag_db3_play_test: record a fresh .db3
# directly from a connected RealSense camera, then verify it's natively
# playable by `ros2 bag play` with byte-identical frames.

import hashlib, subprocess, time
import pyrealsense2 as rs
import pytest, rclpy
from rclpy.qos import HistoryPolicy, QoSProfile
from sensor_msgs.msg import Image


# Topics librealsense's writer produces for the streams we record below,
# mapped to the rs.stream they correspond to. D4xx devices put stereo on
# sensor 0 and RGB on sensor 1; this test's markers cover only those.
TOPICS = {
    "/device_0/sensor_0/Depth_0/image/data": rs.stream.depth,
    "/device_0/sensor_1/Color_0/image/data": rs.stream.color,
}


def get_frame_hash(frames):
    return [hashlib.sha256(f).hexdigest() for f in frames]


def _record_live(db3, duration_s=5.0):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, rs.format.rgb8, 30)
    cfg.enable_record_to_file(db3)
    pipe.start(cfg)
    try:
        deadline = time.time() + duration_s
        while time.time() < deadline:
            pipe.wait_for_frames()
    finally:
        pipe.stop()


def _replay_frames(db3):
    # Ground truth: replay at sensor level (no rs.pipeline syncer in the
    # way) so every frame written to the bag is delivered. Anything
    # missing here that ros2 bag play delivers would be a librealsense
    # write-side bug — out of scope here.
    frames = {t: [] for t in TOPICS}
    stream_to_topic = {s: t for t, s in TOPICS.items()}
    handles = []
    ctx = rs.context()
    dev = ctx.load_device(db3)
    playback = dev.as_playback()
    for sensor in dev.query_sensors():
        profs = [p for p in sensor.get_stream_profiles()
                 if p.stream_type() in stream_to_topic]
        if not profs:
            continue
        # keep_frames=True so the queue buffers everything until we drain
        # it after playback ends, instead of dropping on overflow.
        q = rs.frame_queue(2000, keep_frames=True)
        sensor.open(profs)
        sensor.start(q)
        handles.append((sensor, q))
    try:
        time.sleep(playback.get_duration().total_seconds() + 1.0)
    finally:
        for sensor, _ in handles:
            sensor.stop()
            sensor.close()
    for _, q in handles:
        while True:
            f = q.poll_for_frame()
            if not f:
                break
            t = stream_to_topic.get(f.profile.stream_type())
            if t:
                frames[t].append(bytes(f.get_data()))
    ctx.unload_device(db3)
    return frames


@pytest.mark.d415
@pytest.mark.d435i
@pytest.mark.d455
def test_live_record_db3_play(tmp_path):
    devs = rs.context().query_devices()
    assert list(devs), "no RealSense device connected"
    # Reset the device; prior tests in this pytest run can leave the depth
    # sensor in a state where pipeline.start() opens but no frames arrive.
    devs[0].hardware_reset()
    time.sleep(5)  # USB re-enum + FW boot
    assert subprocess.run(["ros2", "bag", "play", "--help"],
                          capture_output=True).returncode == 0, \
        "ros2 bag play not available (declare ros2bag as <test_depend>)"

    db3 = str(tmp_path / "live.db3")
    _record_live(db3)
    src = _replay_frames(db3)

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("live_db3_subscriber")
    # ros2 bag play publishes RELIABLE by default; KEEP_ALL + large depth
    # prevents subscriber-side drops while we wait for playback to finish.
    qos = QoSProfile(history=HistoryPolicy.KEEP_ALL, depth=1000)
    received = {t: [] for t in TOPICS}
    for t in TOPICS:
        node.create_subscription(
            Image, t,
            lambda m, t=t: received[t].append(hashlib.sha256(bytes(m.data)).hexdigest()),
            qos)

    # The live bag is short (~5 s); --delay holds publishing until DDS has
    # had a chance to wire up the subscriber, otherwise play can drain the
    # bag before discovery completes and we receive zero messages.
    proc = subprocess.Popen(["ros2", "bag", "play", "--delay", "2", db3],
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
    for t in TOPICS:
        src_h = get_frame_hash(src[t])
        rx_h = received[t]
        assert rx_h and (rx_h == src_h[:len(rx_h)] or rx_h == src_h[-len(rx_h):]), \
            f"{t}: content mismatch (got {len(rx_h)} / {len(src_h)} source frames)"
