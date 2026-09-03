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
import ctypes
import glob
import os

import pytest


def librealsense_path():
    ''' Path of the librealsense2 the camera node will load. Resolved through the dynamic
        loader, since the ROS packages install into /opt/ros/<distro>/lib, which is reached
        via LD_LIBRARY_PATH and is not in the ldconfig cache.
    '''
    try:
        ctypes.CDLL('librealsense2.so')
        with open('/proc/self/maps') as maps:
            return next(l.split()[-1] for l in maps if 'librealsense2.so' in l)
    except Exception:
        pass
    libs = [lib for d in os.environ.get('LD_LIBRARY_PATH', '').split(':') if d
            for lib in glob.glob(os.path.join(d, 'librealsense2.so*')) if os.path.getsize(lib)]
    return libs[0] if libs else None


def sdk_plays_compressed_db3():
    ''' Whether the installed librealsense2 can read the /compressed and /compressedDepth
        topics of the .db3 test recordings (librealsense PR #15556). Probes the library for
        the topic suffix rather than its version, since a source build of the development
        branch reports 2.58.0 - lower than the released 2.58.4 that lacks the support.
        Returns True when it cannot tell, so the tests run.
    '''
    path = librealsense_path()
    if not path:
        return True
    try:
        with open(path, 'rb') as lib:
            return b'/compressedDepth' in lib.read()
    except OSError:
        return True


def pytest_collection_modifyitems(config, items):
    if sdk_plays_compressed_db3():
        return
    skip = pytest.mark.skip(reason="installed librealsense2 cannot play compressed .db3 "
                                   "recordings (needs librealsense PR #15556)")
    for item in items:
        if 'rosbag' in item.keywords:
            item.add_marker(skip)
