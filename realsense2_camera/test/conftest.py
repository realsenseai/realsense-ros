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
import subprocess

import pytest


def sdk_plays_compressed_db3():
    ''' Whether the installed librealsense2 can read the /compressed and /compressedDepth
        topics of the .db3 test recordings (librealsense PR #15556). Probes the library for
        the topic suffix rather than its version, since a source build of the development
        branch reports 2.58.0 - lower than the released 2.58.4 that lacks the support.
        Returns True when it cannot tell, so the tests run.
    '''
    try:
        libs = subprocess.run(['ldconfig', '-p'], capture_output=True, text=True).stdout
        path = next(l.rsplit('=>', 1)[-1].strip() for l in libs.splitlines()
                    if 'librealsense2.so' in l and '=>' in l)
        with open(path, 'rb') as lib:
            return b'/compressedDepth' in lib.read()
    except Exception:
        return True


def pytest_collection_modifyitems(config, items):
    if sdk_plays_compressed_db3():
        return
    skip = pytest.mark.skip(reason="installed librealsense2 cannot play compressed .db3 "
                                   "recordings (needs librealsense PR #15556)")
    for item in items:
        if 'rosbag' in item.keywords:
            item.add_marker(skip)
