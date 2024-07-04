// Copyright 2024 Intel Corporation. All Rights Reserved.
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
using namespace realsense2_camera;
using namespace rs2;

/***
 * Implementation of ROS2 Actions based on:
 * ROS2 Actions Design: https://design.ros2.org/articles/actions.html
 * ROS2 Actions Tutorials/Examples: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
*/

// Triggered Calibration Action Struct (Message)
/*
# request
string json "calib run"
---
# result
string calibration
float32 health
---
# feedback
float32 progress
*/

/***
 * A callback for handling new goals (requests) for Triggered Calibration
 * This implementation just accepts all goals with no restriction on the json input
*/
rclcpp_action::GoalResponse BaseRealSenseNode::TriggeredCalibrationHandleGoal(const rclcpp_action::GoalUUID & uuid,
                                                                                 std::shared_ptr<const TriggeredCalibration::Goal> goal)
{
    (void)uuid;  // unused parameter
    ROS_INFO_STREAM("TriggeredCalibrationAction: Received request with json " << goal->json);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/***
 * A callback for handling cancel events
 * This implementation just tells the client that it accepted the cancellation.
*/
rclcpp_action::CancelResponse BaseRealSenseNode::TriggeredCalibrationHandleCancel(const std::shared_ptr<GoalHandleTriggeredCalibration> goal_handle)
{
    (void)goal_handle;  // unused parameter
    ROS_INFO("TriggeredCalibrationAction: Received request to cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}

/***
 * A callback that accepts a new goal (request) and starts processing it.
 * Since the execution is a long-running operation, we spawn off a
 * thread to do the actual work and return from handle_accepted quickly.
*/
void BaseRealSenseNode::TriggeredCalibrationHandleAccepted(const std::shared_ptr<GoalHandleTriggeredCalibration> goal_handle)
{
    using namespace std::placeholders;
    ROS_INFO("TriggeredCalibrationAction: Request accepted");
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&BaseRealSenseNode::TriggeredCalibrationExecute, this, _1), goal_handle}.detach();
}

/***
 * All processing and updates of Triggered Calibration operation
 * are done in this execute method in the new thread called by the 
 * TriggeredCalibrationHandleAccepted() above.
*/
void BaseRealSenseNode::TriggeredCalibrationExecute(const std::shared_ptr<GoalHandleTriggeredCalibration> goal_handle)
{
    ROS_INFO("TriggeredCalibrationAction: Executing...");
    const auto goal = goal_handle->get_goal();  // get the TriggeredCalibration srv struct
    auto feedback = std::make_shared<TriggeredCalibration::Feedback>();
    float & _progress = feedback->progress;
    auto result = std::make_shared<TriggeredCalibration::Result>();

    try
    {
        rs2::auto_calibrated_device ac_dev = _dev.as<auto_calibrated_device>();
        float health = 0.f;  // output health
        int timeout_ms = 120000;  // 2 minutes timout

        auto progress_callback = [&](const float progress) {
                _progress = progress;
                goal_handle->publish_feedback(feedback);
        };

        auto ans = ac_dev.run_on_chip_calibration(goal->json,
                                                &health,
                                                progress_callback,
                                                timeout_ms);

        // the new calibration is the result without the first 3 bytes
        rs2::calibration_table new_calib = std::vector<uint8_t>(ans.begin() + 3, ans.end());

        if (rclcpp::ok() && _progress == 100.0)
        {
            result->calibration = vectorToJsonString(new_calib);
            result->health = health;
            result->success = true;
            goal_handle->succeed(result);
            ROS_INFO("TriggeredCalibrationExecute: Succeded");
        }
        else
        {
            result->calibration = "{}";
            result->success = false;
            result->error_msg = "Canceled";
            goal_handle->canceled(result);
            ROS_WARN("TriggeredCalibrationExecute: Canceled");
        }
    }
    catch(const std::runtime_error& e)
    {
        // exception must have been thrown from run_on_chip_calibration call
        std::string error_msg = "TriggeredCalibrationExecute: Aborted. Error: " + std::string(e.what());
        result->calibration = "{}";
        result->success = false;
        result->error_msg = error_msg;
        goal_handle->abort(result);
        ROS_ERROR(error_msg.c_str());
    }
}
