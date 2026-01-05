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

#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#if defined (ACCELERATE_GPU_WITH_GLSL)

#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <iostream>

#include <librealsense2-gl/rs_processing_gl.hpp> // Include GPU-Processing API


#ifndef PI
#define PI  3.14159265358979323846
#define PI_FL  3.141592f
#endif


class GLwindow
{
public:

    GLwindow(int width, int height, const char* title)
        : _width(width), _height(height)
    {
        glfwInit();
        glfwWindowHint(GLFW_VISIBLE, 0);
        win = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (!win)
            throw std::runtime_error("Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools");
        glfwMakeContextCurrent(win);

        glfwSetWindowUserPointer(win, this);

    }

    ~GLwindow()
    {
        glfwDestroyWindow(win);
        glfwTerminate();
    }

    void close()
    {
        glfwSetWindowShouldClose(win, 1);
    }

    float width() const { return float(_width); }
    float height() const { return float(_height); }

    operator bool()
    {
        auto res = !glfwWindowShouldClose(win);

        glfwPollEvents();

        return res;
    }

    operator GLFWwindow* () { return win; }

private:
    GLFWwindow* win;
    int _width, _height;
};

#endif
