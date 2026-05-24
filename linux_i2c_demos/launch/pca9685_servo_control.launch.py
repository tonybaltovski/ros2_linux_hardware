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
    i2c_bus = LaunchConfiguration('i2c_bus')
    device_id = LaunchConfiguration('device_id')
    pwm_frequency = LaunchConfiguration('pwm_frequency')

    return LaunchDescription([
        DeclareLaunchArgument('i2c_bus', default_value='1',
                              description='I2C bus number.'),
        DeclareLaunchArgument('device_id', default_value='64',
                              description='7-bit I2C address of the PCA9685 (default 0x40 = 64).'),
        DeclareLaunchArgument('pwm_frequency', default_value='50.0',
                              description='PWM frequency in Hz.'),

        Node(
            package='linux_i2c_demos',
            executable='pca9685_servo_control',
            name='pca9685_servo_control',
            output='screen',
            parameters=[{
                'i2c_bus': i2c_bus,
                'device_id': device_id,
                'pwm_frequency': pwm_frequency,
            }],
        ),
    ])
