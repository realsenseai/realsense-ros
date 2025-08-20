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
#include <geometry_msgs/msg/vector3_stamped.hpp>

using namespace realsense2_camera;

void BaseRealSenseNode::restartStaticTransformBroadcaster()
{
    // Since the _static_tf_broadcaster is a latched topic, the old transforms will
    // be alive even if the sensors are dynamically disabled. So, reset the
    // broadcaster everytime and publish the transforms of enabled sensors alone.
    if (_static_tf_broadcaster)
    {
        _static_tf_broadcaster.reset();
    }

    // intra-process do not support latched QoS, so we need to disable intra-process for this topic
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

    _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(_node, 
                                                                                    tf2_ros::StaticBroadcasterQoS(), 
                                                                                    std::move(options));
}

void BaseRealSenseNode::append_static_tf_msg(const rclcpp::Time& t,
                                          const float3& trans,
                                          const tf2::Quaternion& q,
                                          const std::string& from,
                                          const std::string& to)
{
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = from;
    msg.child_frame_id = to;

    // Convert translation vector (x,y,z) (taken from camera extrinsics)
    // from optical cooridnates to ros coordinates
    msg.transform.translation.x = trans.z;
    msg.transform.translation.y = -trans.x;
    msg.transform.translation.z = -trans.y;

    msg.transform.rotation.x = q.getX();
    msg.transform.rotation.y = q.getY();
    msg.transform.rotation.z = q.getZ();
    msg.transform.rotation.w = q.getW();
    _static_tf_msgs.push_back(msg);
}

struct compare_tf_ids
{
    const std::string& frame_id;
    const std::string& child_frame_id;

    // Struct Constructor
    compare_tf_ids(const std::string& frame_id, const std::string& child_frame_id) :
                            frame_id(frame_id), child_frame_id(child_frame_id) {}

    bool operator()(const geometry_msgs::msg::TransformStamped &static_tf_msg) const
    {
        return (static_tf_msg.header.frame_id == frame_id &&
                static_tf_msg.child_frame_id == child_frame_id);
    }
};

void BaseRealSenseNode::erase_static_tf_msg(const std::string& frame_id,
                                            const std::string& child_frame_id)
{
    std::vector<geometry_msgs::msg::TransformStamped>::iterator it;

    // Find whether there is any TF with given 'frame_id' and 'child_frame_id'
    it = std::find_if(_static_tf_msgs.begin(), _static_tf_msgs.end(),
                                        compare_tf_ids(frame_id, child_frame_id));

    // If found, erase that specific TF
    if (it != std::end(_static_tf_msgs))
    {
        _static_tf_msgs.erase(it);
    }
}

void BaseRealSenseNode::publishExtrinsicsTopic(const stream_index_pair& sip, const rs2_extrinsics& ex){
    // Publish extrinsics topic:
    Extrinsics msg = rsExtrinsicsToMsg(ex);
    if (_extrinsics_publishers.find(sip) != _extrinsics_publishers.end())
    {
        _extrinsics_publishers[sip]->publish(msg);
    }
}

tf2::Quaternion BaseRealSenseNode::rotationMatrixToQuaternion(const float rotation[9]) const
{
    Eigen::Matrix3f m;
    // We need to be careful about the order, as RS2 rotation matrix is
    // column-major, while Eigen::Matrix3f expects row-major.
    m << rotation[0], rotation[3], rotation[6],
         rotation[1], rotation[4], rotation[7],
         rotation[2], rotation[5], rotation[8];
    Eigen::Quaternionf q(m);
    return tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
}

void BaseRealSenseNode::calcAndAppendTransformMsgs(const rs2::stream_profile& profile, 
                                                   const rs2::stream_profile& base_profile)
{
    // Transform base to stream
    stream_index_pair sip(profile.stream_type(), profile.stream_index());

    // rotation quaternion from ROS CS to Optical CS
    tf2::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0, -M_PI/2);  // R,P,Y rotations over the original axes

    // zero rotation quaternion, used for IMU
    tf2::Quaternion zero_rot_quaternions;
    zero_rot_quaternions.setRPY(0, 0, 0);

    // zero translation, used for moving from ROS frame to its correspending Optical frame
    float3 zero_trans{0, 0, 0};

    rclcpp::Time transform_ts_ = _node.now();

    // extrinsic from A to B is the position of A relative to B
    // TF from A to B is the transformation to be done on A to get to B
    // so, we need to calculate extrinsics in two opposite ways, one for extrinsic topic
    // and the second is for transformation topic (TF)
    rs2_extrinsics normal_ex;  // used to for extrinsics topic
    rs2_extrinsics tf_ex; // used for TF

    try
    {
        normal_ex = base_profile.get_extrinsics_to(profile);
        tf_ex = profile.get_extrinsics_to(base_profile);
    }
    catch (std::exception& e)
    {
        if (!strcmp(e.what(), "Requested extrinsics are not available!"))
        {
            ROS_WARN_STREAM("(" << rs2_stream_to_string(base_profile.stream_type()) << ", "
                                << base_profile.stream_index() << ") -> ("
                                << rs2_stream_to_string(profile.stream_type()) << ", "
                                << profile.stream_index() << "): "
                                << e.what()
                                << " : using unity as default.");
            normal_ex = rs2_extrinsics({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}});
            tf_ex = normal_ex;
        }
        else
        {
            throw e;
        }
    }

    // publish normal extrinsics e.g. /camera/extrinsics/depth_to_color
    publishExtrinsicsTopic(sip, normal_ex);

    // Representing Rotations with Quaternions
    // see https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Using_quaternions_as_rotations for reference

    // Q defines a quaternion rotation from <base profile CS> to <current profile CS>
    // for example, rotation from depth to infra2
    auto Q = rotationMatrixToQuaternion(tf_ex.rotation);

    float3 trans{tf_ex.translation[0], tf_ex.translation[1], tf_ex.translation[2]};

    if(profile.stream_type() == RS2_STREAM_LABELED_POINT_CLOUD ||
       profile.stream_type() == RS2_STREAM_OCCUPANCY)
    {
        // e.g. tf_ex will include the 
        // Extrinsics from "Labeled Point Cloud" to "Depth" which is:
        // - Rotation matrix from Depth (Optical CS) to Labeled Point Cloud (Robot CS)
        // - Translation vector in the Optical CS
        
        // We take optical rotation (quaternion_optical) and rotate it by Q (which rotates from optical to lpcl/occupancy robot CS)
        // Order is important (from left to right)
        Q = Q * quaternion_optical;

        // We still use the same translation vector (trans) since it is already in the optical CS
        // and inside append_static_tf_msg we always convert translation to Robot CS
        append_static_tf_msg(transform_ts_, trans, Q, BASE_FRAME_ID, FRAME_ID(sip));
    }
    else
    {
        // Rotation order is important (start from left to right):
        // 1. quaternion_optical.inverse() [ROS -> Optical]
        // 2. Q [Optical -> Optical] (usually no rotation, but might be very small rotations between sensors, like from Depth to Color)
        // 3. quaternion_optical [Optical -> ROS]
        // We do all these products since we want to finish in ROS CS, while Q is a rotation from optical to optical,
        // and cant be used directly in ROS TF without this combination 
        Q = quaternion_optical * Q * quaternion_optical.inverse();

        // Also here, the translation vector is in the Optical CS, and we convert it to ROS CS inside append_static_tf_msg
        append_static_tf_msg(transform_ts_, trans, Q, BASE_FRAME_ID, FRAME_ID(sip));
    }

    // Transform stream frame from ROS CS to optical CS and publish it
    // We are using zero translation vector here, since no translation between frame and optical_frame, but only rotation
    append_static_tf_msg(transform_ts_, zero_trans, quaternion_optical, FRAME_ID(sip), OPTICAL_FRAME_ID(sip));

    if (profile.is<rs2::video_stream_profile>() &&
                profile.stream_type() != RS2_STREAM_DEPTH &&
                profile.stream_index() == 1)
    {
        append_static_tf_msg(transform_ts_, trans, Q, BASE_FRAME_ID, ALIGNED_DEPTH_TO_FRAME_ID(sip));
        append_static_tf_msg(transform_ts_, zero_trans, quaternion_optical, 
                                                ALIGNED_DEPTH_TO_FRAME_ID(sip), OPTICAL_FRAME_ID(sip));
    }

    if ((_imu_sync_method > imu_sync_method::NONE) && (profile.stream_type() == RS2_STREAM_GYRO))
    {
        append_static_tf_msg(transform_ts_, zero_trans, zero_rot_quaternions, FRAME_ID(sip), IMU_FRAME_ID);
        append_static_tf_msg(transform_ts_, zero_trans, quaternion_optical, IMU_FRAME_ID, IMU_OPTICAL_FRAME_ID);
    }
}

void BaseRealSenseNode::eraseTransformMsgs(const stream_index_pair& sip, const rs2::stream_profile& profile)
{
    std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);

    erase_static_tf_msg(BASE_FRAME_ID, FRAME_ID(sip));
    erase_static_tf_msg(FRAME_ID(sip), OPTICAL_FRAME_ID(sip));

    if (profile.is<rs2::video_stream_profile>() &&
                profile.stream_type() != RS2_STREAM_DEPTH &&
                profile.stream_index() == 1)
    {
        erase_static_tf_msg(BASE_FRAME_ID, ALIGNED_DEPTH_TO_FRAME_ID(sip));
        erase_static_tf_msg(ALIGNED_DEPTH_TO_FRAME_ID(sip), OPTICAL_FRAME_ID(sip));
    }

    if ((_imu_sync_method > imu_sync_method::NONE) && (profile.stream_type() == RS2_STREAM_GYRO))
    {
        erase_static_tf_msg(FRAME_ID(sip), IMU_FRAME_ID);
        erase_static_tf_msg(IMU_FRAME_ID, IMU_OPTICAL_FRAME_ID);
    }
}

void BaseRealSenseNode::publishStaticTransforms()
{
    restartStaticTransformBroadcaster();

    _static_tf_broadcaster->sendTransform(_static_tf_msgs);
}

void BaseRealSenseNode::publishDynamicTransforms()
{
    if (!_dynamic_tf_broadcaster)
    {
        _dynamic_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(_node);
    }

    // Publish transforms for the cameras
    std::unique_lock<std::mutex> lock(_publish_dynamic_tf_mutex);
    while (rclcpp::ok() && _is_running && _tf_publish_rate > 0)
    {
        _cv_tf.wait_for(lock, std::chrono::milliseconds((int)(1000.0/_tf_publish_rate)), 
                                        [&]{return (!(_is_running && _tf_publish_rate > 0));});
        {
            std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);
            rclcpp::Time t = _node.now();
            try
            {
                for(auto& msg : _static_tf_msgs)
                    msg.header.stamp = t;
                _dynamic_tf_broadcaster->sendTransform(_static_tf_msgs);
            }
            catch(const std::exception& e)
            {
                ROS_ERROR_STREAM("Error publishing dynamic transforms: " << e.what());
            }
        }
    }
}

void BaseRealSenseNode::startDynamicTf()
{
    if (!_publish_tf)
    {
        ROS_WARN("Since the param 'publish_tf' is set to 'false',"
                "the value set on the param 'tf_publish_rate' won't have any effect");
        return;
    }
    if (_tf_publish_rate > 0)
    {
        // Start publishing dynamic TF, if the param 'tf_publish_rate' is set to > 0.0 Hz
        ROS_WARN("Publishing dynamic camera transforms (/tf) at %g Hz", _tf_publish_rate);
        if (!_tf_t)
        {
            _tf_t = std::make_shared<std::thread>([this]()
            {
                publishDynamicTransforms();
            });
        }
    }
    else
    {
        if (_tf_t && _tf_t->joinable())
        {
            // Stop publishing dynamic TF by resetting the '_tf_t' thread and '_dynamic_tf_broadcaster'
            _tf_t->join();
            _tf_t.reset();
            _dynamic_tf_broadcaster.reset();
            ROS_WARN("Stopped publishing dynamic camera transforms (/tf)");
        }
        else
        {
            // '_tf_t' thread is not running currently. i.e, dynamic tf is not getting broadcasted.
            ROS_WARN("Currently not publishing dynamic camera transforms (/tf). "
                     "To start publishing it, set the 'tf_publish_rate' param to > 0.0 Hz ");
        }
    }
}

