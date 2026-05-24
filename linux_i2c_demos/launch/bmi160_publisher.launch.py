# Copyright 2026 Tony Baltovski
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("i2c_bus", default_value="1", description="I2C bus number."),
            DeclareLaunchArgument(
                "device_id",
                default_value="104",
                description="I2C address (default 0x68 = 104; 0x69 = 105 if SDO=V_DDIO).",
            ),
            DeclareLaunchArgument(
                "publish_rate", default_value="100.0", description="Publish frequency in Hz."
            ),
            DeclareLaunchArgument(
                "accel_range_g", default_value="2", description="Accel full-scale: 2/4/8/16 g."
            ),
            DeclareLaunchArgument(
                "gyro_range_dps",
                default_value="2000",
                description="Gyro full-scale: 125/250/500/1000/2000 dps.",
            ),
            DeclareLaunchArgument(
                "frame_id", default_value="bmi160", description="frame_id stamped on messages."
            ),
            Node(
                package="linux_i2c_demos",
                executable="bmi160_publisher",
                name="bmi160_publisher",
                output="screen",
                parameters=[
                    {
                        "i2c_bus": LaunchConfiguration("i2c_bus"),
                        "device_id": LaunchConfiguration("device_id"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                        "accel_range_g": LaunchConfiguration("accel_range_g"),
                        "gyro_range_dps": LaunchConfiguration("gyro_range_dps"),
                        "frame_id": LaunchConfiguration("frame_id"),
                    }
                ],
            ),
        ]
    )
