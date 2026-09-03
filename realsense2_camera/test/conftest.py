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
import functools
import subprocess

import pytest


@functools.lru_cache(maxsize=1)
def sdk_plays_compressed_db3():
    ''' Whether the installed librealsense2 can play back the compressed .db3 recordings
        the rosbag tests use.

        Those recordings keep their frames on /compressed and /compressedDepth topics, which
        librealsense only learned to read in PR #15556. The librealsense2 packages on the ROS
        servers can still predate it, and the reader then logs "unknown message type" for every
        metadata message and delivers no frames.

        This probes the installed library for the topic suffix instead of comparing versions,
        because a source build of the development branch reports 2.58.0 - lower than the
        released 2.58.4 that lacks the support - so a version floor would skip the tests
        exactly where they do work. Fails open: if the library cannot be located, run them.
    '''
    try:
        ldconfig = subprocess.run(['ldconfig', '-p'], capture_output=True, text=True).stdout
    except (OSError, subprocess.SubprocessError):
        return True
    for line in ldconfig.splitlines():
        if 'librealsense2.so' in line and '=>' in line:
            try:
                with open(line.split('=>')[-1].strip(), 'rb') as lib:
                    return b'/compressedDepth' in lib.read()
            except OSError:
                return True
    return True


def pytest_collection_modifyitems(config, items):
    if sdk_plays_compressed_db3():
        return
    skip_rosbag = pytest.mark.skip(
        reason="installed librealsense2 cannot play compressed .db3 recordings "
               "(needs librealsense PR #15556)")
    for item in items:
        if 'rosbag' in item.keywords:
            item.add_marker(skip_rosbag)
