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
            DeclareLaunchArgument('i2c_bus', default_value='1', description='I2C bus number.'),
            DeclareLaunchArgument(
                'device_id',
                default_value='25',
                description='I2C address (default 0x19 = 25).',
            ),
            DeclareLaunchArgument(
                'publish_rate', default_value='10.0', description='Publish frequency in Hz.'
            ),
            DeclareLaunchArgument(
                'frame_id', default_value='hmc6343', description='frame_id stamped on messages.'
            ),
            Node(
                package='linux_i2c_demos',
                executable='hmc6343_publisher',
                name='hmc6343_publisher',
                output='screen',
                parameters=[
                    {
                        'i2c_bus': LaunchConfiguration('i2c_bus'),
                        'device_id': LaunchConfiguration('device_id'),
                        'publish_rate': LaunchConfiguration('publish_rate'),
                        'frame_id': LaunchConfiguration('frame_id'),
                    }
                ],
            ),
        ]
    )
