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

#ifdef RVIZ_RGBD_PLUGIN
#include "realsense_rviz_plugin.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_rendering/render_window.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreRectangle2D.h>
#include <OgreResourceGroupManager.h>
#include <OgreAxisAlignedBox.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPainter>
#include <QLabel>

#include <pluginlib/class_list_macros.hpp>

namespace realsense_rviz_plugin {

static int g_instance_counter = 0;

RealsenseRvizPlugin::RealsenseRvizPlugin()
{
  rgb_texture_ = std::make_unique<ROSImageTexture>();
  depth_texture_ = std::make_unique<ROSImageTexture>();
}

RealsenseRvizPlugin::~RealsenseRvizPlugin() = default;

void RealsenseRvizPlugin::onInitialize()
{
  rviz_common::RosTopicDisplay<realsense2_camera_msgs::msg::RGBD>::onInitialize();

  topic_property_->setMessageType("realsense2_camera_msgs/msg/RGBD");
  topic_property_->setDescription("RealSense RGBD topic to subscribe to");

  QObject::connect(topic_property_, SIGNAL(changed()), this, SLOT(updateTopic()));

  display_property_group_ = new rviz_common::properties::Property(
    "Display Options", QVariant(), "Options for displaying RGB and depth images", this);

  show_rgb_property_ = new rviz_common::properties::BoolProperty(
    "Show RGB", true, "Show the RGB image", display_property_group_,
    SLOT(updateDisplayOptions()), this);

  show_depth_property_ = new rviz_common::properties::BoolProperty(
    "Show Depth", true, "Show the depth image", display_property_group_,
    SLOT(updateDisplayOptions()), this);

  show_overlay_property_ = new rviz_common::properties::BoolProperty(
    "Show Depth Overlay", false, "Show depth image as overlay on RGB",
    display_property_group_, SLOT(updateDisplayOptions()), this);

  flip_images_property_ = new rviz_common::properties::BoolProperty(
    "Rotate 180°", false, "Rotate both RGB and depth images 180 degrees",
    display_property_group_, SLOT(updateDisplayOptions()), this);

  depth_property_group_ = new rviz_common::properties::Property(
    "Depth Visualization", QVariant(), "Settings for depth image visualization", this);

  depth_min_property_ = new rviz_common::properties::FloatProperty(
    "Min Range (m)", 0.1f, "Minimum depth value for colormap (meters)",
    depth_property_group_, SLOT(updateDepthDisplayOptions()), this);

  depth_max_property_ = new rviz_common::properties::FloatProperty(
    "Max Range (m)", 5.0f, "Maximum depth value for colormap (meters)",
    depth_property_group_, SLOT(updateDepthDisplayOptions()), this);

  use_jet_colormap_property_ = new rviz_common::properties::BoolProperty(
    "Use Jet Colormap", true, "Apply jet colormap to depth image",
    depth_property_group_, SLOT(updateDepthDisplayOptions()), this);

  overlay_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Overlay Alpha", 0.5f, "Alpha transparency for depth overlay (0.0 - 1.0)",
    depth_property_group_, SLOT(updateDepthDisplayOptions()), this);
  overlay_alpha_property_->setMin(0.0);
  overlay_alpha_property_->setMax(1.0);

  setupRenderPanel();
  setupScreenRectangles();
}

void RealsenseRvizPlugin::setupRenderPanel()
{
  render_panel_ = std::make_unique<rviz_common::RenderPanel>();
  render_panel_->initialize(context_);
  setAssociatedWidget(render_panel_.get());

  Ogre::SceneManager* scene_manager = context_->getSceneManager();

  std::string camera_name = "RealsenseCamera_" + std::to_string(g_instance_counter++);
  render_camera_ = scene_manager->createCamera(camera_name);
  render_camera_->setNearClipDistance(0.01f);
  render_camera_->setFarClipDistance(10.0f);
  render_camera_->setPosition(Ogre::Vector3(0, 0, 10));
  render_camera_->lookAt(Ogre::Vector3(0, 0, 0));
  render_camera_->setAutoAspectRatio(true);

  render_panel_->getRenderWindow()->setupSceneAfterInit(
    [this](Ogre::SceneNode* scene_node) {
      scene_node_ = scene_node;
    });
}

void RealsenseRvizPlugin::setupScreenRectangles()
{
  static int material_count = 0;
  material_count++;

  // RGB rectangle (left half by default)
  rgb_screen_rect_ = std::make_unique<Ogre::Rectangle2D>(true);
  rgb_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  rgb_screen_rect_->setCorners(-1.0f, 1.0f, 0.0f, -1.0f);

  const std::string rgb_material_name =
    "RealsenseRvizPluginRGBMaterial" + std::to_string(material_count);
  rgb_material_ = Ogre::MaterialManager::getSingleton().create(
    rgb_material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  rgb_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_REPLACE);
  rgb_material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  rgb_material_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
  rgb_material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  rgb_material_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
  {
    Ogre::TextureUnitState* tu =
      rgb_material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureFiltering(Ogre::TextureFilterOptions::TFO_BILINEAR);
    tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  }
  rgb_screen_rect_->setMaterial(rgb_material_);

  // Depth rectangle (right half by default)
  depth_screen_rect_ = std::make_unique<Ogre::Rectangle2D>(true);
  depth_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  depth_screen_rect_->setCorners(0.0f, 1.0f, 1.0f, -1.0f);

  const std::string depth_material_name =
    "RealsenseRvizPluginDepthMaterial" + std::to_string(material_count);
  depth_material_ = Ogre::MaterialManager::getSingleton().create(
    depth_material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  depth_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_REPLACE);
  depth_material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  depth_material_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
  depth_material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  depth_material_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
  {
    Ogre::TextureUnitState* tu =
      depth_material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureFiltering(Ogre::TextureFilterOptions::TFO_BILINEAR);
    tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  }
  depth_screen_rect_->setMaterial(depth_material_);

  // Infinite bounds
  Ogre::AxisAlignedBox aab;
  aab.setInfinite();
  rgb_screen_rect_->setBoundingBox(aab);
  depth_screen_rect_->setBoundingBox(aab);

  // Attach when scene is ready
  render_panel_->getRenderWindow()->setupSceneAfterInit(
    [this](Ogre::SceneNode* scene_node) {
      scene_node_ = scene_node;
      scene_node->attachObject(rgb_screen_rect_.get());
      scene_node->attachObject(depth_screen_rect_.get());
    });
}

void RealsenseRvizPlugin::onEnable()
{
  rviz_common::RosTopicDisplay<realsense2_camera_msgs::msg::RGBD>::onEnable();
}

void RealsenseRvizPlugin::onDisable()
{
  rviz_common::RosTopicDisplay<realsense2_camera_msgs::msg::RGBD>::onDisable();
  clear();
}

void RealsenseRvizPlugin::reset()
{
  rviz_common::RosTopicDisplay<realsense2_camera_msgs::msg::RGBD>::reset();
  clear();
}

void RealsenseRvizPlugin::clear()
{
  current_rgb_msg_.reset();
  current_depth_msg_.reset();
  if (rgb_texture_) { rgb_texture_->clear(); }
  if (depth_texture_) { depth_texture_->clear(); }
}

void RealsenseRvizPlugin::updateTopic()
{
  // Re-subscribe using the base, then reset our textures
  rviz_common::RosTopicDisplay<realsense2_camera_msgs::msg::RGBD>::updateTopic();
  clear();
}

void RealsenseRvizPlugin::updateDepthDisplayOptions()
{
  if (current_depth_msg_) {
    processDepthImage();
  }
}

void RealsenseRvizPlugin::updateDisplayOptions()
{
  const bool show_rgb = show_rgb_property_->getBool();
  const bool show_depth = show_depth_property_->getBool();
  const bool show_overlay =
    show_overlay_property_->getBool() && show_depth && show_rgb;

  if (rgb_screen_rect_) { rgb_screen_rect_->setVisible(show_rgb); }
  if (depth_screen_rect_) { depth_screen_rect_->setVisible(show_depth && !show_overlay); }

  if (rgb_screen_rect_ && depth_screen_rect_) {
    if (show_overlay) {
      // Fullscreen RGB, depth blended into it
      rgb_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
      rgb_screen_rect_->setVisible(true);
    } else if (show_rgb && show_depth) {
      // Split view
      rgb_screen_rect_->setCorners(-1.0f, 1.0f, 0.0f, -1.0f);
      depth_screen_rect_->setCorners(0.0f, 1.0f, 1.0f, -1.0f);
      if (depth_material_) {
        depth_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_REPLACE);
      }
    } else if (show_rgb && !show_depth) {
      rgb_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
    } else if (!show_rgb && show_depth) {
      depth_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
    }
  }

  if (show_overlay && current_depth_msg_ && current_rgb_msg_) {
    processDepthImage();
  }
  context_->queueRender();
}

void RealsenseRvizPlugin::processMessage(
  const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr msg)
{
  if (!msg) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Message", "Received null message");
    return;
  }

  try {
    if (msg->rgb.data.empty()) {
      setStatus(rviz_common::properties::StatusProperty::Error, "RGB", "Image data is empty");
      return;
    }
    if (msg->depth.data.empty()) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Depth", "Image data is empty");
      return;
    }

    current_rgb_msg_ = std::make_shared<sensor_msgs::msg::Image>(msg->rgb);
    current_depth_msg_ = std::make_shared<sensor_msgs::msg::Image>(msg->depth);

    setStatus(
      rviz_common::properties::StatusProperty::Ok, "Images",
      QString("%1x%2 RGB, %3x%4 depth")
        .arg(msg->rgb.width)
        .arg(msg->rgb.height)
        .arg(msg->depth.width)
        .arg(msg->depth.height));
    setStatus(rviz_common::properties::StatusProperty::Ok, "RGB", "RGB image OK");
    setStatus(rviz_common::properties::StatusProperty::Ok, "Depth", "Depth image OK");

    context_->queueRender();
  } catch (const std::exception& e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Message",
      QString("Error processing message: %1").arg(e.what()));
    return;
  }
}

void RealsenseRvizPlugin::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  if (current_rgb_msg_) {
    try {
      if (flip_images_property_->getBool()) {
        cv_bridge::CvImagePtr cv_rgb =
          cv_bridge::toCvCopy(current_rgb_msg_, sensor_msgs::image_encodings::BGR8);
        cv::Mat rotated;
        cv::rotate(cv_rgb->image, rotated, cv::ROTATE_180);
        cv_rgb->image = rotated;

        auto rotated_msg = cv_bridge::CvImage(
            current_rgb_msg_->header, cv_rgb->encoding, cv_rgb->image)
            .toImageMsg();

        if (rgb_texture_->update(rotated_msg)) {
          if (rgb_texture_->getTexture()) {
            rgb_material_->getTechnique(0)->getPass(0)
              ->getTextureUnitState(0)->setTexture(rgb_texture_->getTexture());
          }
        }
      } else {
        if (rgb_texture_->update(current_rgb_msg_)) {
          if (rgb_texture_->getTexture()) {
            rgb_material_->getTechnique(0)->getPass(0)
              ->getTextureUnitState(0)->setTexture(rgb_texture_->getTexture());
          }
        }
      }
    } catch (const std::exception& e) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "RGB",
        QString("Exception updating RGB texture: %1").arg(e.what()));
    }
  }

  if (current_depth_msg_) {
    try {
      processDepthImage();
    } catch (const std::exception& e) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Depth",
        QString("Exception updating depth texture: %1").arg(e.what()));
    }
  }

  if (current_rgb_msg_ || current_depth_msg_) {
    setStatus(rviz_common::properties::StatusProperty::Ok, "Images", "Images received");
  } else {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Images", "No images received");
  }
}

bool RealsenseRvizPlugin::processDepthImage()
{
  if (!current_depth_msg_ || !depth_texture_) {
    return false;
  }

  const bool show_overlay =
    show_overlay_property_->getBool() &&
    show_depth_property_->getBool() &&
    show_rgb_property_->getBool();

  const bool flip_images = flip_images_property_->getBool();

  try {
    cv_bridge::CvImagePtr cv_depth;
    cv::Mat colored_depth;

    const double min_depth = depth_min_property_->getFloat();
    const double max_depth = depth_max_property_->getFloat();
    const bool use_jet = use_jet_colormap_property_->getBool();
    const float alpha = overlay_alpha_property_->getFloat();

    if (current_depth_msg_->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        current_depth_msg_->encoding == sensor_msgs::image_encodings::MONO16)
    {
      cv_depth = cv_bridge::toCvCopy(current_depth_msg_, sensor_msgs::image_encodings::TYPE_16UC1);
      if (flip_images) {
        cv::Mat rotated;
        cv::rotate(cv_depth->image, rotated, cv::ROTATE_180);
        cv_depth->image = rotated;
      }

      cv::Mat float_image;
      cv_depth->image.convertTo(float_image, CV_32F, 0.001); // mm -> m

      cv::Mat depth_mask = float_image > 0;
      float_image.setTo(max_depth, float_image > max_depth);
      float_image.setTo(min_depth, (float_image < min_depth) & depth_mask);

      cv::Mat normalized_image = 255.0 * (float_image - min_depth) / (max_depth - min_depth);
      normalized_image.convertTo(normalized_image, CV_8U);
      normalized_image.setTo(0, ~depth_mask);

      if (use_jet) {
        cv::applyColorMap(normalized_image, colored_depth, cv::COLORMAP_JET);
        cv::Mat invalid_mask = ~depth_mask;
        colored_depth.setTo(cv::Vec3b(0,0,0), invalid_mask);
      } else {
        cv::cvtColor(normalized_image, colored_depth, cv::COLOR_GRAY2BGR);
      }
    }
    else if (current_depth_msg_->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      cv_depth = cv_bridge::toCvCopy(current_depth_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
      if (flip_images) {
        cv::Mat rotated;
        cv::rotate(cv_depth->image, rotated, cv::ROTATE_180);
        cv_depth->image = rotated;
      }

      cv::Mat depth_mask = cv_depth->image > 0;
      cv_depth->image.setTo(max_depth, cv_depth->image > max_depth);
      cv_depth->image.setTo(min_depth, (cv_depth->image < min_depth) & depth_mask);

      cv::Mat float_image = cv_depth->image;
      cv::Mat normalized_image = 255.0 * (float_image - min_depth) / (max_depth - min_depth);
      normalized_image.convertTo(normalized_image, CV_8U);
      normalized_image.setTo(0, ~depth_mask);

      if (use_jet) {
        cv::applyColorMap(normalized_image, colored_depth, cv::COLORMAP_JET);
        cv::Mat invalid_mask = ~depth_mask;
        colored_depth.setTo(cv::Vec3b(0,0,0), invalid_mask);
      } else {
        cv::cvtColor(normalized_image, colored_depth, cv::COLOR_GRAY2BGR);
      }
    }
    else {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Depth",
        QString("Unsupported depth encoding: %1")
          .arg(QString::fromStdString(current_depth_msg_->encoding)));
      return false;
    }

    if (show_overlay && current_rgb_msg_) {
      // Blend into RGB
      cv_bridge::CvImagePtr cv_rgb =
        cv_bridge::toCvCopy(current_rgb_msg_, sensor_msgs::image_encodings::BGR8);
      if (flip_images) {
        cv::Mat rotated;
        cv::rotate(cv_rgb->image, rotated, cv::ROTATE_180);
        cv_rgb->image = rotated;
      }

      if (cv_rgb->image.size() != colored_depth.size()) {
        cv::resize(colored_depth, colored_depth, cv_rgb->image.size(), 0, 0, cv::INTER_LINEAR);
      }

      cv::Mat alpha_mask(colored_depth.size(), CV_32FC3, cv::Scalar(alpha, alpha, alpha));
      cv::Mat inv_alpha_mask(colored_depth.size(), CV_32FC3,
                             cv::Scalar(1.0f - alpha, 1.0f - alpha, 1.0f - alpha));

      cv::Mat depth_32f, rgb_32f;
      colored_depth.convertTo(depth_32f, CV_32FC3, 1.0/255.0);
      cv_rgb->image.convertTo(rgb_32f, CV_32FC3, 1.0/255.0);

      // Valid where depth != black (after colormap)
      cv::Mat depth_zero_mask = (colored_depth == cv::Vec3b(0,0,0));
      std::vector<cv::Mat> channels;
      cv::split(depth_zero_mask, channels);
      cv::Mat depth_valid_mask = ~(channels[0] & channels[1] & channels[2]);

      cv::multiply(depth_32f, alpha_mask, depth_32f);
      cv::multiply(rgb_32f, inv_alpha_mask, rgb_32f);

      cv::Mat blended_32f = rgb_32f.clone();
      cv::Mat roi;
      depth_valid_mask.convertTo(roi, CV_8U);
      cv::add(depth_32f, rgb_32f, blended_32f, roi);

      cv::Mat blended_8u;
      blended_32f.convertTo(blended_8u, CV_8UC3, 255.0);

      auto blended_msg = cv_bridge::CvImage(
          current_rgb_msg_->header, sensor_msgs::image_encodings::BGR8, blended_8u)
          .toImageMsg();

      if (rgb_texture_->update(blended_msg)) {
        if (rgb_texture_->getTexture()) {
          rgb_material_->getTechnique(0)->getPass(0)
            ->getTextureUnitState(0)->setTexture(rgb_texture_->getTexture());
        }
      }
      setStatus(rviz_common::properties::StatusProperty::Ok, "Depth", "Depth image OK");
      return true;
    } else {
      // Show processed depth on the depth rectangle
      cv_depth->image = colored_depth;
      cv_depth->encoding = sensor_msgs::image_encodings::BGR8;

      auto processed_depth_msg = cv_bridge::CvImage(
          current_depth_msg_->header, cv_depth->encoding, cv_depth->image)
          .toImageMsg();

      if (depth_texture_->update(processed_depth_msg)) {
        if (depth_texture_->getTexture()) {
          depth_material_->getTechnique(0)->getPass(0)
            ->getTextureUnitState(0)->setTexture(depth_texture_->getTexture());
        }
        setStatus(rviz_common::properties::StatusProperty::Ok, "Depth", "Depth image OK");
        return true;
      }
    }
  } catch (const std::exception& e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Depth",
      QString("Exception processing depth image: %1").arg(e.what()));
  }

  return false;
}

}  // namespace realsense_rviz_plugin

PLUGINLIB_EXPORT_CLASS(realsense_rviz_plugin::RealsenseRvizPlugin, rviz_common::Display)
#endif // RVIZ_RGBD_PLUGIN