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

#include <pointcloud_filter.h>
#include <fstream>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "device_pointcloud_utils.h"


using namespace realsense2_camera;


PointcloudFilter::PointcloudFilter(std::shared_ptr<rs2::filter> filter, RosNodeBase& node, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled):
    NamedFilter(filter, parameters, logger, is_enabled, false),
    _node(node),
    _allow_no_texture_points(ALLOW_NO_TEXTURE_POINTS),
    _ordered_pc(ORDERED_PC),
    _source(POINTCLOUD_SOURCE_HOST)
    {
        setParameters();
    }

void PointcloudFilter::setParameters()
{
    std::string module_name = create_graph_resource_name(rs2_to_ros(_filter->get_info(RS2_CAMERA_INFO_NAME)));
    std::string param_name(module_name + "." + "allow_no_texture_points");
    _params.getParameters()->setParamT(param_name, _allow_no_texture_points);
    _parameters_names.push_back(param_name);

    param_name = module_name + "." + std::string("ordered_pc");
    _params.getParameters()->setParamT(param_name, _ordered_pc);
    _parameters_names.push_back(param_name);

    // Read once at startup (BaseRealSenseNode::setupDevicePointCloud validates it); no runtime change.
    param_name = module_name + "." + std::string("source");
    rcl_interfaces::msg::ParameterDescriptor source_descriptor;
    source_descriptor.description = "Where the pointcloud is built. Available options are:\n"
                                    + POINTCLOUD_SOURCE_HOST + " : on the host, from the Depth stream (default)\n"
                                    + POINTCLOUD_SOURCE_DEVICE + " : by the device (D585 Depth Mapping sensor), requires enable_depth:=false";
    _source = _params.getParameters()->setParam<std::string>(param_name, POINTCLOUD_SOURCE_HOST,
                                                             std::function<void(const rclcpp::Parameter&)>(), source_descriptor);
    _parameters_names.push_back(param_name);

    param_name = module_name + "." + std::string("pointcloud_qos");
    rcl_interfaces::msg::ParameterDescriptor crnt_descriptor;
    crnt_descriptor.description = "Available options are:\n" + list_available_qos_strings();
    _pointcloud_qos = _params.getParameters()->setParam<std::string>(param_name, DEFAULT_QOS, [this](const rclcpp::Parameter& parameter)
            {
                try
                {
                    qos_string_to_qos(parameter.get_value<std::string>());
                    _pointcloud_qos = parameter.get_value<std::string>();
                    ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
                }
                catch(const std::exception& e)
                {
                    ROS_ERROR_STREAM("Given value, " << parameter.get_value<std::string>() << " is unknown. Set ROS param back to: " << _pointcloud_qos);
                    _params.getParameters()->queueSetRosValue(parameter.get_name(), _pointcloud_qos);
                }
            }, crnt_descriptor);
    _parameters_names.push_back(param_name);
    NamedFilter::setParameters([this](const rclcpp::Parameter& )
        {
            setPublisher();
        });
}

void PointcloudFilter::setPublisher()
{
    std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
    if ((_is_enabled) && (!_pointcloud_publisher))
    {
        _pointcloud_publisher = _node.create_publisher<sensor_msgs::msg::PointCloud2>("~/depth/color/points",
                                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_string_to_qos(_pointcloud_qos)),
                                            qos_string_to_qos(_pointcloud_qos)));
    }
    else if ((!_is_enabled) && (_pointcloud_publisher))
    {
        _pointcloud_publisher.reset();
    }
}

void reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n)
{
    size_t i;

    for (i=0; i < n; ++i)
        dst[n-1-i] = src[i];

}

void PointcloudFilter::Publish(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset, const std::string& frame_id)
{
    {
        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
        if ((!_pointcloud_publisher) || (!(_pointcloud_publisher->get_subscription_count())))
            return;
    }
    
    rs2_stream texture_source_id = static_cast<rs2_stream>(_filter->get_option(rs2_option::RS2_OPTION_STREAM_FILTER));
    bool use_texture = texture_source_id != RS2_STREAM_ANY;
    static int warn_count(0);
    static const int DISPLAY_WARN_NUMBER(15);
    rs2::frameset::iterator texture_frame_itr = frameset.end();
    
    if (use_texture)
    {
        std::set<rs2_format> available_formats{ rs2_format::RS2_FORMAT_RGB8, rs2_format::RS2_FORMAT_Y8, rs2_format::RS2_FORMAT_Z16 };

        texture_frame_itr = std::find_if(frameset.begin(), frameset.end(), [&texture_source_id, &available_formats] (rs2::frame f)
                                {return (rs2_stream(f.get_profile().stream_type()) == texture_source_id) &&
                                            (available_formats.find(f.get_profile().format()) != available_formats.end()); });
        if (texture_frame_itr == frameset.end())
        {
            warn_count++;
            std::string texture_source_name = _filter->get_option_value_description(rs2_option::RS2_OPTION_STREAM_FILTER, static_cast<float>(texture_source_id));
            ROS_WARN_STREAM_COND(warn_count == DISPLAY_WARN_NUMBER, "No stream match for pointcloud chosen texture " << texture_source_name);
            return;
        }
        warn_count = 0;
    } 
    else {
        warn_count++;
        std::string texture_source_name = _filter->get_option_value_description(
            rs2_option::RS2_OPTION_STREAM_FILTER,
            static_cast<float>(texture_source_id)
        );
        ROS_WARN_STREAM_COND(
            warn_count == DISPLAY_WARN_NUMBER,
            "No matching stream for texture '" << texture_source_name
            << "'. Set 'pointcloud.stream_profile' to 'depth(1)', 'color(2)', or 'infrared(3)' to enable textured pointclouds."
        );        
    }

    int texture_width(0), texture_height(0);
    int num_colors(0);

    const rs2::vertex* vertex = pc.get_vertices();
    const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();

    rs2_intrinsics depth_intrin = pc.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    sensor_msgs::msg::PointCloud2::UniquePtr msg_pointcloud = std::make_unique<sensor_msgs::msg::PointCloud2>();

    sensor_msgs::PointCloud2Modifier modifier(*msg_pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pc.size());
    if (_ordered_pc)
    {
        msg_pointcloud->width = depth_intrin.width;
        msg_pointcloud->height = depth_intrin.height;
        msg_pointcloud->is_dense = false;
    }

    vertex = pc.get_vertices();
    size_t valid_count(0);
    if (use_texture)
    {
        rs2::video_frame texture_frame = (*texture_frame_itr).as<rs2::video_frame>();
        texture_width = texture_frame.get_width();
        texture_height = texture_frame.get_height();
        num_colors = texture_frame.get_bytes_per_pixel();
        uint8_t* color_data = (uint8_t*)texture_frame.get_data();
        std::string format_str;
        switch(texture_frame.get_profile().format())
        {
            case RS2_FORMAT_RGB8:
                format_str = "rgb";
                break;
            case RS2_FORMAT_Y8:
                format_str = "intensity";
                break;
            case RS2_FORMAT_Z16:
                // Depth can't be used as color texture — skip coloring
                format_str = "";  // Don't add any color field
                break;
            default:
                throw std::runtime_error("Unhandled texture format passed in pointcloud " + std::to_string(texture_frame.get_profile().format()));
        }        
        msg_pointcloud->point_step = addPointField(*msg_pointcloud, format_str.c_str(), 1, sensor_msgs::msg::PointField::FLOAT32, msg_pointcloud->point_step);
        msg_pointcloud->row_step = msg_pointcloud->width * msg_pointcloud->point_step;
        msg_pointcloud->data.resize(msg_pointcloud->height * msg_pointcloud->row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(*msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(*msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(*msg_pointcloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(*msg_pointcloud, format_str);
        color_point = pc.get_texture_coordinates();

        float color_pixel[2];
        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++, color_point++)
        {
            float i(color_point->u);
            float j(color_point->v);
            bool valid_color_pixel(i >= 0.f && i <=1.f && j >= 0.f && j <=1.f);
            bool valid_pixel(vertex->z > 0 && (valid_color_pixel || _allow_no_texture_points));
            if (valid_pixel || _ordered_pc)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;

                if (valid_color_pixel)
                {
                    color_pixel[0] = i * texture_width;
                    color_pixel[1] = j * texture_height;
                    int pixx = static_cast<int>(color_pixel[0]);
                    int pixy = static_cast<int>(color_pixel[1]);
                    int offset = (pixy * texture_width + pixx) * num_colors;
                    reverse_memcpy(&(*iter_color), color_data+offset, num_colors);  // PointCloud2 order of rgb is bgr.
                }
                ++iter_x; ++iter_y; ++iter_z;
                ++iter_color;
                ++valid_count;
            }
        }
    }
    else
    {
        msg_pointcloud->row_step = msg_pointcloud->width * msg_pointcloud->point_step;
        msg_pointcloud->data.resize(msg_pointcloud->height * msg_pointcloud->row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(*msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(*msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(*msg_pointcloud, "z");

        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++)
        {
            bool valid_pixel(vertex->z > 0);
            if (valid_pixel || _ordered_pc)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;

                ++iter_x; ++iter_y; ++iter_z;
                ++valid_count;
            }
        }
    }
    msg_pointcloud->header.stamp = t;
    msg_pointcloud->header.frame_id = frame_id;
    if (!_ordered_pc)
    {
        msg_pointcloud->width = valid_count;
        msg_pointcloud->height = 1;
        msg_pointcloud->is_dense = true;
        modifier.resize(valid_count);
    }
    {
        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
        if (_pointcloud_publisher)
            _pointcloud_publisher->publish(std::move(msg_pointcloud));
    }
}

void PointcloudFilter::PublishDevice(rs2::points pc, const rclcpp::Time& t, const std::string& frame_id, float clip_distance)
{
    {
        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
        if ((!_pointcloud_publisher) || (!(_pointcloud_publisher->get_subscription_count())))
            return;
    }

    // The device cloud has no intrinsics and no texture coordinates: it is the bare W x H XYZ32F vertex array,
    // so the geometry comes from the stream profile.
    const rs2::video_stream_profile profile = pc.get_profile().as<rs2::video_stream_profile>();
    const size_t width = static_cast<size_t>(profile.width());
    const size_t height = static_cast<size_t>(profile.height());
    if (profile.format() != RS2_FORMAT_XYZ32F || pc.size() != width * height)
    {
        RCLCPP_WARN_THROTTLE(_logger, *_node.get_clock(), 5000,
                             "Device point cloud frame dropped: format %s, %zu points (expected XYZ32F, %zu x %zu points)",
                             rs2_format_to_string(profile.format()), pc.size(), width, height);
        return;
    }

    static_assert(sizeof(rs2::vertex) == devicepcl::VERTEX_SIZE, "rs2::vertex must be a packed float3");
    const bool ordered_pc(_ordered_pc);   // may be changed at runtime: use one value for the whole frame

    sensor_msgs::msg::PointCloud2::UniquePtr msg_pointcloud = std::make_unique<sensor_msgs::msg::PointCloud2>();

    // Same fields as the host "xyz" cloud (x@0, y@4, z@8, FLOAT32; point_step 16).
    const char* field_names[] = {"x", "y", "z"};
    for (uint32_t i = 0; i < 3; ++i)
    {
        sensor_msgs::msg::PointField field;
        field.name = field_names[i];
        field.offset = i * static_cast<uint32_t>(sizeof(float));
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg_pointcloud->fields.push_back(field);
    }
    msg_pointcloud->is_bigendian = false;
    msg_pointcloud->point_step = devicepcl::POINT_STEP;

    const size_t count = devicepcl::repackXyz(reinterpret_cast<const float*>(pc.get_vertices()), pc.size(),
                                              ordered_pc, clip_distance, msg_pointcloud->data);
    if (ordered_pc)
    {
        msg_pointcloud->width = static_cast<uint32_t>(width);
        msg_pointcloud->height = static_cast<uint32_t>(height);
        msg_pointcloud->is_dense = false;
    }
    else
    {
        msg_pointcloud->width = static_cast<uint32_t>(count);
        msg_pointcloud->height = 1;
        msg_pointcloud->is_dense = true;
    }
    msg_pointcloud->row_step = msg_pointcloud->width * msg_pointcloud->point_step;
    msg_pointcloud->header.stamp = t;
    msg_pointcloud->header.frame_id = frame_id;
    {
        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
        if (_pointcloud_publisher)
            _pointcloud_publisher->publish(std::move(msg_pointcloud));
    }
}
