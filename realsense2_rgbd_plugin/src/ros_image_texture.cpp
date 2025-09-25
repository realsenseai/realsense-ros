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

#include "ros_image_texture.hpp"

#include <iostream>

#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <OgreTextureManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreResourceGroupManager.h>
#include <OgreRenderTexture.h>

namespace realsense_rviz_plugin
{

uint32_t ROSImageTexture::texture_count_ = 0;

ROSImageTexture::ROSImageTexture()
: width_(0)
, height_(0)
, texture_width_(0)
, texture_height_(0)
{
  texture_name_ = "ROSImageTexture" + std::to_string(texture_count_++);
}

ROSImageTexture::~ROSImageTexture()
{
  clear();
}

void ROSImageTexture::clear()
{
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_name_);
    texture_.reset();
  }
  width_ = 0;
  height_ = 0;
  texture_width_ = 0;
  texture_height_ = 0;
}

bool ROSImageTexture::update(const sensor_msgs::msg::Image::ConstSharedPtr& image)
{
  if (!image) {
    std::cerr << "ROSImageTexture: Received null image message" << std::endl;
    return false;
  }

  cv_bridge::CvImageConstPtr cv_image;

  // Convert to BGR8 or MONO8
  try {
    if (sensor_msgs::image_encodings::isColor(image->encoding)) {
      cv_image = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
    } else if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
               image->encoding == sensor_msgs::image_encodings::MONO16)
    {
      // Normalize 16-bit to mono8
      cv_bridge::CvImageConstPtr original = cv_bridge::toCvShare(image);
      cv::Mat normalized;

      double min_val = 0, max_val = 0;
      cv::minMaxLoc(original->image, &min_val, &max_val);
      if (max_val > 0) {
        cv::convertScaleAbs(original->image, normalized, 255.0 / max_val);
      } else {
        normalized = cv::Mat::zeros(original->image.size(), CV_8UC1);
      }

      cv_image = std::make_shared<cv_bridge::CvImage>(
        original->header, sensor_msgs::image_encodings::MONO8, normalized);
    } else {
      // Mono8 and friends
      cv_image = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO8);
    }
  } catch (const cv_bridge::Exception&) {
    return false;
  }

  const int width = cv_image->image.cols;
  const int height = cv_image->image.rows;

  if (!texture_ || static_cast<int>(texture_width_) != width ||
      static_cast<int>(texture_height_) != height)
  {
    if (texture_) {
      Ogre::TextureManager::getSingleton().remove(texture_name_);
    }

    texture_name_ = "ROSImageTexture" + std::to_string(texture_count_++);

    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      width, height,
      0, // mipmaps
      Ogre::PF_BYTE_BGRA,
      Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    if (!texture_) {
      std::cerr << "ROSImageTexture: Failed to create texture!" << std::endl;
      return false;
    }

    texture_width_ = width;
    texture_height_ = height;
  }

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
  if (!pixel_buffer) {
    std::cerr << "ROSImageTexture: Failed to get pixel buffer" << std::endl;
    return false;
  }

  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox& pixel_box = pixel_buffer->getCurrentLock();

  uint8_t* dest_ptr = static_cast<uint8_t*>(pixel_box.data);
  const size_t dest_pitch = pixel_box.rowPitch * 4;  // BGRA

  if (cv_image->encoding == sensor_msgs::image_encodings::BGR8) {
    for (int y = 0; y < cv_image->image.rows; ++y) {
      for (int x = 0; x < cv_image->image.cols; ++x) {
        const cv::Vec3b& src = cv_image->image.at<cv::Vec3b>(y, x);
        uint8_t* dst = dest_ptr + y * dest_pitch + x * 4;
        dst[0] = src[0]; // B
        dst[1] = src[1]; // G
        dst[2] = src[2]; // R
        dst[3] = 255;    // A
      }
    }
  } else if (cv_image->encoding == sensor_msgs::image_encodings::MONO8) {
    for (int y = 0; y < cv_image->image.rows; ++y) {
      for (int x = 0; x < cv_image->image.cols; ++x) {
        const uint8_t gray = cv_image->image.at<uint8_t>(y, x);
        uint8_t* dst = dest_ptr + y * dest_pitch + x * 4;
        dst[0] = gray; // B
        dst[1] = gray; // G
        dst[2] = gray; // R
        dst[3] = 255;  // A
      }
    }
  }

  pixel_buffer->unlock();

  width_ = width;
  height_ = height;
  return true;
}

void ROSImageTexture::ensure_texture_created()
{
  if (texture_ || width_ == 0 || height_ == 0) {
    return;
  }

  try {
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      width_, height_,
      0,
      Ogre::PF_BYTE_BGRA,
      Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  } catch (const Ogre::Exception& e) {
    std::cerr << "Failed to create texture: " << e.what() << std::endl;
    width_ = 0;
    height_ = 0;
    texture_.reset();
  }
}

Ogre::PixelFormat ROSImageTexture::getOgrePixelFormat(const std::string& encoding)
{
  using sensor_msgs::image_encodings::BGR8;
  using sensor_msgs::image_encodings::RGB8;
  using sensor_msgs::image_encodings::MONO8;
  using sensor_msgs::image_encodings::TYPE_8UC1;
  using sensor_msgs::image_encodings::MONO16;
  using sensor_msgs::image_encodings::TYPE_16UC1;
  using sensor_msgs::image_encodings::TYPE_32FC1;

  if (encoding == BGR8 || encoding == RGB8) {
    return Ogre::PF_BYTE_BGRA;
  } else if (encoding == MONO8 || encoding == TYPE_8UC1) {
    return Ogre::PF_BYTE_L;
  } else if (encoding == MONO16 || encoding == TYPE_16UC1) {
    return Ogre::PF_L16;
  } else if (encoding == TYPE_32FC1) {
    return Ogre::PF_FLOAT32_R;
  }

  return Ogre::PF_BYTE_BGRA;
}

}  // namespace realsense_rviz_plugin
