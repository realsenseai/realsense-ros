// Copyright 2023 RealSense, Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../include/base_realsense_node.h"
#include <chrono>

using namespace realsense2_camera;
using namespace std::chrono_literals;

#if defined (ACCELERATE_GPU_WITH_GLSL)

void BaseRealSenseNode::initOpenGLProcessing(bool use_gpu_processing)
{
    // Once we have a window, initialize GL module
    // Pass our window to enable sharing of textures between processed frames and the window
    // The "use_gpu_processing" is going to control if we will use CPU or GPU for data processing
    rs2::gl::init_processing(_app, use_gpu_processing);
    if (use_gpu_processing)
        rs2::gl::init_rendering();

    _timer = _node.create_wall_timer(1000ms, std::bind(&BaseRealSenseNode::glfwPollEventCallback, this));
}

void BaseRealSenseNode::glfwPollEventCallback()
{
    // Must poll the GLFW events perodically, else window will hang or crash
    glfwPollEvents();
}

void BaseRealSenseNode::shutdownOpenGLProcessing()
{
    rs2::gl::shutdown_rendering();
    rs2::gl::shutdown_processing();
}

#endif
