// Copyright 2025 RealSense, Inc. All Rights Reserved.
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

#include <cstdint>
#include <memory>
#include <string>

#include <sensor_msgs/msg/image.hpp>

#include <OgreTexture.h>  // Ogre::TexturePtr

namespace realsense_rviz_plugin {

/**
 * @brief Minimal helper to convert sensor_msgs::Image into an Ogre texture.
 *        Supports BGR8 and MONO8 (others are preprocessed before update()).
 */
class ROSImageTexture
{
public:
  ROSImageTexture();
  ~ROSImageTexture();

  void clear();

  // Update the texture contents from a ROS image. Returns true if updated.
  bool update(const sensor_msgs::msg::Image::ConstSharedPtr& image);

  // Create the Ogre texture object if needed (no-op if already created).
  void ensure_texture_created();

  // Access the underlying Ogre texture (may be null if not created/updated yet).
  Ogre::TexturePtr getTexture() const { return texture_; }

  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }

  // Utility to map encodings (not strictly required by caller).
  static Ogre::PixelFormat getOgrePixelFormat(const std::string& encoding);

private:
  static uint32_t texture_count_;
  std::string texture_name_;

  uint32_t width_;
  uint32_t height_;
  uint32_t texture_width_;
  uint32_t texture_height_;

  Ogre::TexturePtr texture_;
};

}  // namespace realsense_rviz_plugin
