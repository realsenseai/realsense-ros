# Copyright 2024 RealSense, Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""ParameterHandler for handling parameter-related MQTT requests."""
import json

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters

from .service_handler import ServiceHandler


class ParameterHandler(ServiceHandler):
    """
    Handler for parameter-related MQTT requests.

    This class processes MQTT requests related to setting and getting parameters
    for a camera, and interacts with ROS services to handle these requests.
    """

    def __init__(self, mqtt_ros_node):
        """
        Initialize the ParameterHandler.

        Args:
            mqtt_ros_node: Instance of the MQTTBridgeNode.
        """
        self.mqtt_ros_node = mqtt_ros_node
        self.parameter_type_map = {
            'float': ParameterType.PARAMETER_DOUBLE,
            'int': ParameterType.PARAMETER_INTEGER,
            'integer': ParameterType.PARAMETER_INTEGER,
            'string': ParameterType.PARAMETER_STRING,
            'bool': ParameterType.PARAMETER_BOOL,
            'boolean': ParameterType.PARAMETER_BOOL
        }
        self.type_map = {
            ParameterType.PARAMETER_BOOL: 'bool',
            ParameterType.PARAMETER_INTEGER: 'integer',
            ParameterType.PARAMETER_DOUBLE: 'double',
            ParameterType.PARAMETER_STRING: 'string'
        }

    def handle_set_param_request(self, mqtt_request):
        """
        Handle the set parameter MQTT request.

        Args:
            mqtt_request (dict): The MQTT request message containing the
            camera namespace, camera name, parameter name, parameter value,
            and parameter type.
        """
        self.mqtt_ros_node.ROS_DEBUG('set_param_request message received')

        camera_namespace = mqtt_request['camera_namespace']
        camera_name = mqtt_request['camera_name']
        parameter_name = mqtt_request['parameter_name']
        parameter_value = mqtt_request['parameter_value']
        parameter_type = mqtt_request['parameter_type'].lower()
        service_name = f'/{camera_namespace}/{camera_name}/set_parameters'

        ros_client_set_parameter = self.create_ros_client(SetParameters, service_name)
        if not ros_client_set_parameter:
            return

        parameter_value_obj = self.create_parameter_value(parameter_type, parameter_value)
        if not parameter_value_obj:
            self.mqtt_ros_node.ROS_ERROR(f'unsupported parameter_type: {parameter_type}')
            return

        set_parameter_request = SetParameters.Request()
        set_parameter_request.parameters.append(
            Parameter(name=parameter_name, value=parameter_value_obj))

        future = ros_client_set_parameter.call_async(set_parameter_request)

        self.wait_for_future(future)

        mqtt_response = self.create_set_param_response(camera_namespace, camera_name, future.result())

        self.mqtt_ros_node.mqtt_client.publish('set_parameter_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('set_param_response message sent')
        ros_client_set_parameter.destroy()

    def handle_get_param_request(self, mqtt_request):
        """
        Handle the get parameter MQTT request.

        Args:
            mqtt_request (dict): The MQTT request message containing the
            camera namespace, camera name, and parameter name.
        """
        self.mqtt_ros_node.ROS_DEBUG('get_param_request message received')

        camera_namespace = mqtt_request['camera_namespace']
        camera_name = mqtt_request['camera_name']
        parameter_name = mqtt_request['parameter_name']
        service_name = f'/{camera_namespace}/{camera_name}/get_parameters'

        ros_client_get_parameter = self.create_ros_client(GetParameters, service_name)
        if not ros_client_get_parameter:
            return

        ros_request = GetParameters.Request()
        ros_request.names.append(parameter_name)

        future = ros_client_get_parameter.call_async(ros_request)

        self.wait_for_future(future)

        mqtt_response = self.create_get_param_response(camera_namespace, camera_name, parameter_name, future.result())

        self.mqtt_ros_node.mqtt_client.publish('get_parameter_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('get_parameter_response message sent')
        ros_client_get_parameter.destroy()

    def create_parameter_value(self, parameter_type, parameter_value):
        """
        Create a ParameterValue object based on the type and value.

        Args:
            parameter_type (str): The type of the parameter.
            parameter_value: The value of the parameter.

        Returns:
            A ParameterValue object.
        """
        if parameter_type in self.parameter_type_map:
            p = ParameterValue(type=self.parameter_type_map[parameter_type])
            if parameter_type == 'float':
                p.double_value = float(parameter_value)
            elif parameter_type in ['int', 'integer']:
                p.integer_value = int(parameter_value)
            elif parameter_type == 'string':
                p.string_value = parameter_value
            elif parameter_type == 'bool':
                p.bool_value = parameter_value.lower() == 'true'
            return p
        return None

    def create_set_param_response(self, camera_namespace, camera_name, ros_response):
        """
        Create a response for the set parameter request.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            ros_response: The ROS response.

        Returns:
            A dictionary representing the MQTT response.
        """
        mqtt_response = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name
        }

        result = ros_response.results[0]

        if result.successful:
            self.mqtt_ros_node.ROS_DEBUG('Parameter set successfully')
            mqtt_response['success'] = True
            mqtt_response['error_msg'] = ''
        else:
            self.mqtt_ros_node.ROS_WARN('Failed to set parameter')
            mqtt_response['success'] = False
            mqtt_response['error_msg'] = result.reason

        return mqtt_response

    def create_get_param_response(self, camera_namespace, camera_name, parameter_name, ros_response):
        """
        Create a response for the get parameter request.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            parameter_name (str): The name of the parameter.
            ros_response: The ROS response.

        Returns:
            A dictionary representing the MQTT response.
        """
        mqtt_response = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'parameter_name': parameter_name,
            'success': True,
            'error_msg': ''
        }

        result = ros_response.values[0]
        result_type = self.type_map.get(result.type, 'unsupported')
        if result_type != 'unsupported':
            self.mqtt_ros_node.ROS_DEBUG('Parameter retrieved successfully')
            mqtt_response['parameter_type'] = result_type
            mqtt_response['parameter_value'] = str(
                getattr(result, f'{result_type}_value'))
        else:
            self.mqtt_ros_node.ROS_WARN('Failed to retrieve parameter')
            mqtt_response['success'] = False
            mqtt_response['error_msg'] = 'unsupported type'

        return mqtt_response
