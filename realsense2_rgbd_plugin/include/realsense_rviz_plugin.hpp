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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>

#include <OgreRectangle2D.h>
#include <OgreMaterial.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneNode.h>

#include <QTimer>

#include "ros_image_texture.hpp"

namespace realsense_rviz_plugin {

class RealsenseRvizPlugin
  : public rviz_common::RosTopicDisplay<realsense2_camera_msgs::msg::RGBD>
{
  Q_OBJECT

public:
  RealsenseRvizPlugin();
  ~RealsenseRvizPlugin() override;

protected:
  void onInitialize() override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;
  void processMessage(const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr msg) override;
  void update(float, float) override;
  void updateTopic() override;

private Q_SLOTS:
  void updateDepthDisplayOptions();
  void updateDisplayOptions();
  void checkMessageStatus();

private:
  void clear();
  void setupScreenRectangles();
  void setupRenderPanel();
  bool processDepthImage();

  // Common render panel
  std::unique_ptr<rviz_common::RenderPanel> render_panel_;

  // Ogre camera/viewport/scene
  Ogre::Viewport* viewport_{nullptr};
  Ogre::Camera* render_camera_{nullptr};
  Ogre::SceneNode* scene_node_{nullptr};

  // Ogre rectangles & materials
  std::unique_ptr<Ogre::Rectangle2D> rgb_screen_rect_;
  Ogre::MaterialPtr rgb_material_;
  std::unique_ptr<Ogre::Rectangle2D> depth_screen_rect_;
  Ogre::MaterialPtr depth_material_;

  // Textures
  std::unique_ptr<ROSImageTexture> rgb_texture_;
  std::unique_ptr<ROSImageTexture> depth_texture_;

  // Current messages
  sensor_msgs::msg::Image::SharedPtr current_rgb_msg_;
  sensor_msgs::msg::Image::SharedPtr current_depth_msg_;

  // Property groups
  rviz_common::properties::Property* depth_property_group_{nullptr};
  rviz_common::properties::Property* display_property_group_{nullptr};

  // Display options
  rviz_common::properties::BoolProperty* show_rgb_property_{nullptr};
  rviz_common::properties::BoolProperty* show_depth_property_{nullptr};
  rviz_common::properties::BoolProperty* show_overlay_property_{nullptr};
  rviz_common::properties::BoolProperty* flip_images_property_{nullptr};

  // Depth display properties
  rviz_common::properties::FloatProperty* depth_min_property_{nullptr};
  rviz_common::properties::FloatProperty* depth_max_property_{nullptr};
  rviz_common::properties::BoolProperty* use_jet_colormap_property_{nullptr};
  rviz_common::properties::FloatProperty* overlay_alpha_property_{nullptr};

  // (Optional) message monitoring (not used, kept for future)
  std::unique_ptr<QTimer> message_monitor_timer_;
  int message_count_{0};
  rclcpp::Time last_message_time_;
};

}  // namespace realsense_rviz_plugin
