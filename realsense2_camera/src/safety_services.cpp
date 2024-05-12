#include "../include/base_realsense_node.h"
#include <fstream>

using namespace realsense2_camera;
using namespace rs2;

void BaseRealSenseNode::publishSafetyServices()
{
    _safety_preset_read_srv = _node.create_service<realsense2_camera_msgs::srv::SafetyPresetRead>(
            "~/safety_preset_read",
            [&](const realsense2_camera_msgs::srv::SafetyPresetRead::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::SafetyPresetRead::Response::SharedPtr res)
                        {SafetyPresetReadService(req, res);});

    _safety_preset_write_srv = _node.create_service<realsense2_camera_msgs::srv::SafetyPresetWrite>(
            "~/safety_preset_write",
            [&](const realsense2_camera_msgs::srv::SafetyPresetWrite::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::SafetyPresetWrite::Response::SharedPtr res)
                        {SafetyPresetWriteService(req, res);});


}

void BaseRealSenseNode::SafetyPresetReadService(const realsense2_camera_msgs::srv::SafetyPresetRead::Request::SharedPtr req,
    realsense2_camera_msgs::srv::SafetyPresetRead::Response::SharedPtr res){
    try
    {
        int index = req->index;
    
        if (index < 0 || index > 63)
        {
            res->success = false;
            res->error_message = "Invalid index. Index must be between 0 and 63.";
        }
        rs2_safety_preset sp = _safety_sensor->as<rs2::safety_sensor>().get_safety_preset(index);
        json json_data = safety_preset_to_json(sp);
        res->result = json_data.dump();
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::SafetyPresetWriteService(const realsense2_camera_msgs::srv::SafetyPresetWrite::Request::SharedPtr req,
    realsense2_camera_msgs::srv::SafetyPresetWrite::Response::SharedPtr res){
    try
    {
        int index = req->index;
    
        if (index < 0 || index > 63)
        {
            res->success = false;
            res->error_message = "Invalid index. Index must be between 0 and 63.";
        }
        json json_data = json::parse(req->preset);
        rs2_safety_preset sp = json_to_safety_preset(json_data);
        _safety_sensor->as<rs2::safety_sensor>().set_safety_preset(index, sp);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

    /*

    See: realsense2_camera_msgs/srv/SafetyPreset.srv

    inputs:
    string operation // read/write
    string preset_to_write '' // json to write if operation is write, ignored otherwise
    uint8 index 0 // index to read from or to write to

    outputs:
    bool success // true/false
    string error_message
    string result // preset read in json format if operation is read, ignored otherwise

    */
/*
    try
    {
        std::string operation = req->operation;
        int index = req->index;
    
        if (index < 0 || index > 63)
        {
            res->success = false;
            res->error_message = "Invalid index. Index must be between 0 and 63.";
        }
        else if (operation == "read")
        { 
            rs2_safety_preset sp = (_safety_sensor->as<rs2::safety_sensor>()).get_safety_preset(index);
            json json_data = safety_preset_to_json(sp);
            res->result = json_data.dump();
            res->success = true;
        }
        else if (operation == "write")
        {
            json json_data = json::parse(req->preset_to_write);
            rs2_safety_preset sp = json_to_safety_preset(json_data);
            _safety_sensor->as<rs2::safety_sensor>().set_safety_preset(index, sp);
            res->result = "";
            res->success = true;
        }
        else 
        {
            res->success = false;
            res->result = "";
            res->error_message = "Invalid operation. Operation must be read or write.";
        }
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}
*/

json BaseRealSenseNode::safety_preset_to_json(rs2_safety_preset &sp)
{
    json json_data;
    auto &&rotation = json_data["safety_preset"]["platform_config"]["transformation_link"]["rotation"];
    rotation[0][0] = sp.platform_config.transformation_link.rotation.x.x;
    rotation[0][1] = sp.platform_config.transformation_link.rotation.x.y;
    rotation[0][1] = sp.platform_config.transformation_link.rotation.x.z;
    rotation[1][0] = sp.platform_config.transformation_link.rotation.y.x;
    rotation[1][1] = sp.platform_config.transformation_link.rotation.y.y;
    rotation[1][2] = sp.platform_config.transformation_link.rotation.y.z;
    rotation[2][0] = sp.platform_config.transformation_link.rotation.z.x;
    rotation[2][1] = sp.platform_config.transformation_link.rotation.z.y;
    rotation[2][2] = sp.platform_config.transformation_link.rotation.z.z;

    auto &&translation = json_data["safety_preset"]["platform_config"]["transformation_link"]["translation"];
    translation[0] = sp.platform_config.transformation_link.translation.x;
    translation[1] = sp.platform_config.transformation_link.translation.y;
    translation[2] = sp.platform_config.transformation_link.translation.z;

    json_data["safety_preset"]["platform_config"]["robot_height"] = sp.platform_config.robot_height;

    auto &&safety_zones = json_data["safety_preset"]["safety_zones"];
    for (int safety_zone_index = 0; safety_zone_index < 2; safety_zone_index++)
    {
        std::string zone_type = (safety_zone_index == 0) ? "danger_zone" : "warning_zone";
        for (int zone_polygon_index = 0; zone_polygon_index < 4; zone_polygon_index++)
        {
            std::string p = "p" + std::to_string(zone_polygon_index);
            safety_zones[zone_type]["zone_polygon"][p]["x"] = sp.safety_zones[safety_zone_index].zone_polygon[zone_polygon_index].x;
            safety_zones[zone_type]["zone_polygon"][p]["y"] = sp.safety_zones[safety_zone_index].zone_polygon[zone_polygon_index].y;
        }
        safety_zones[zone_type]["safety_trigger_confidence"] = sp.safety_zones[safety_zone_index].safety_trigger_confidence;
    }

    auto &&masking_zones = json_data["safety_preset"]["masking_zones"];
    for (int masking_zone_index = 0; masking_zone_index < 8; masking_zone_index++)
    {
        std::string masking_zone_index_str = std::to_string(masking_zone_index);
        for (int region_of_interests_index = 0; region_of_interests_index < 4; region_of_interests_index++)
        {
            std::string p = "p" + std::to_string(region_of_interests_index);
            masking_zones[masking_zone_index_str]["region_of_interesets"][p]["i"] = sp.masking_zones[masking_zone_index].region_of_interests[region_of_interests_index].i;
            masking_zones[masking_zone_index_str]["region_of_interesets"][p]["j"] = sp.masking_zones[masking_zone_index].region_of_interests[region_of_interests_index].j;
        }
        masking_zones[masking_zone_index_str]["attributes"] = sp.masking_zones[masking_zone_index].attributes;
    }

    auto &&environment = json_data["safety_preset"]["environment"];
    environment["safety_trigger_duration"] = sp.environment.safety_trigger_duration;
    environment["linear_velocity"] = sp.environment.linear_velocity;
    environment["angular_velocity"] = sp.environment.angular_velocity;
    environment["payload_weight"] = sp.environment.payload_weight;
    environment["surface_inclination"] = sp.environment.surface_inclination;
    environment["surface_height"] = sp.environment.surface_height;
    environment["diagnostic_zone_fill_rate_threshold"] = sp.environment.diagnostic_zone_fill_rate_threshold;
    environment["floor_fill_threshold"] = sp.environment.floor_fill_threshold;
    environment["depth_fill_threshold"] = sp.environment.depth_fill_threshold;
    environment["diagnostic_zone_height_median_threshold"] = sp.environment.diagnostic_zone_height_median_threshold;
    environment["vision_hara_persistency"] = sp.environment.vision_hara_persistency;
    //memcpy((void *)(environment["crypto_signature"].get<std::array<int, 32>>().data()), sp.environment.crypto_signature, 32);
    return json_data;
}


rs2_safety_preset BaseRealSenseNode::json_to_safety_preset(json &json_data)
{
    rs2_safety_preset sp;
    auto &&rotation = json_data["safety_preset"]["platform_config"]["transformation_link"]["rotation"];
    sp.platform_config.transformation_link.rotation.x.x = rotation[0][0];
    sp.platform_config.transformation_link.rotation.x.y = rotation[0][1];
    sp.platform_config.transformation_link.rotation.x.z = rotation[0][1];
    sp.platform_config.transformation_link.rotation.y.x = rotation[1][0];
    sp.platform_config.transformation_link.rotation.y.y = rotation[1][1];
    sp.platform_config.transformation_link.rotation.y.z = rotation[1][2];
    sp.platform_config.transformation_link.rotation.z.x = rotation[2][0];
    sp.platform_config.transformation_link.rotation.z.y = rotation[2][1];
    sp.platform_config.transformation_link.rotation.z.z = rotation[2][2];

    auto &&translation = json_data["safety_preset"]["platform_config"]["transformation_link"]["translation"];
    sp.platform_config.transformation_link.translation.x = translation[0];
    sp.platform_config.transformation_link.translation.y = translation[1];
    sp.platform_config.transformation_link.translation.z = translation[2];

    sp.platform_config.robot_height = json_data["safety_preset"]["platform_config"]["robot_height"];

    auto &&safety_zones = json_data["safety_preset"]["safety_zones"];
    for (int safety_zone_index = 0; safety_zone_index < 2; safety_zone_index++)
    {
        std::string zone_type = (safety_zone_index == 0) ? "danger_zone" : "warning_zone";
        for (int zone_polygon_index = 0; zone_polygon_index < 4; zone_polygon_index++)
        {
            std::string p = "p" + std::to_string(zone_polygon_index);
            sp.safety_zones[safety_zone_index].zone_polygon[zone_polygon_index].x = safety_zones[zone_type]["zone_polygon"][p]["x"];
            sp.safety_zones[safety_zone_index].zone_polygon[zone_polygon_index].y = safety_zones[zone_type]["zone_polygon"][p]["y"];
        }
        sp.safety_zones[safety_zone_index].safety_trigger_confidence = safety_zones[zone_type]["safety_trigger_confidence"];
    }

    auto &&masking_zones = json_data["safety_preset"]["masking_zones"];
    for (int masking_zone_index = 0; masking_zone_index < 8; masking_zone_index++)
    {
        std::string masking_zone_index_str = std::to_string(masking_zone_index);
        for (int region_of_interests_index = 0; region_of_interests_index < 4; region_of_interests_index++)
        {
            std::string p = "p" + std::to_string(region_of_interests_index);
            sp.masking_zones[masking_zone_index].region_of_interests[region_of_interests_index].i = masking_zones[masking_zone_index_str]["region_of_interesets"][p]["i"];
            sp.masking_zones[masking_zone_index].region_of_interests[region_of_interests_index].j = masking_zones[masking_zone_index_str]["region_of_interesets"][p]["j"];
        }
        sp.masking_zones[masking_zone_index].attributes = masking_zones[masking_zone_index_str]["attributes"];
    }

    auto &&environment = json_data["safety_preset"]["environment"];
    sp.environment.safety_trigger_duration = environment["safety_trigger_duration"];
    sp.environment.linear_velocity = environment["linear_velocity"];
    sp.environment.angular_velocity = environment["angular_velocity"];
    sp.environment.payload_weight = environment["payload_weight"];
    sp.environment.surface_inclination = environment["surface_inclination"];
    sp.environment.surface_height = environment["surface_height"];
    sp.environment.diagnostic_zone_fill_rate_threshold = environment["diagnostic_zone_fill_rate_threshold"];
    sp.environment.floor_fill_threshold = environment["floor_fill_threshold"];
    sp.environment.depth_fill_threshold = environment["depth_fill_threshold"];
    sp.environment.diagnostic_zone_height_median_threshold = environment["diagnostic_zone_height_median_threshold"];
    sp.environment.vision_hara_persistency = environment["vision_hara_persistency"];
    //memcpy(sp.environment.crypto_signature, (void *)(environment["crypto_signature"].get<std::array<int, 32>>().data()), 32);
    return sp;
}

