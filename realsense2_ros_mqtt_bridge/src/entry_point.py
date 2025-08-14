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
"""Main module for running the MQTT <-> ROS bridge node."""
import rclpy

from .mqtt_bridge_node import MQTTBridgeNode


def main(args=None):
    """
    Initialize and run the MQTTBridgeNode.

    This function initializes the ROS2 system, creates an instance of the
    MQTTBridgeNode, spins the node, and shuts down the ROS2 system when the
    node is finished.

    Args:
        args: Command-line arguments (default is None).
    """
    try:
        rclpy.init(args=args)
        mqtt_bridge_node = MQTTBridgeNode()
        rclpy.spin(mqtt_bridge_node)
        mqtt_bridge_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(e)
        print('Retry launching the node.')
        exit(1)


if __name__ == '__main__':
    main()
