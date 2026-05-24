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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    display_type = LaunchConfiguration('display_type')
    i2c_bus = LaunchConfiguration('i2c_bus')
    scroll_period = LaunchConfiguration('scroll_period')
    rows = LaunchConfiguration('rows')
    columns = LaunchConfiguration('columns')

    is_lcm1602 = IfCondition(PythonExpression(['"', display_type, '" == "lcm1602"']))
    is_ssd1306 = IfCondition(PythonExpression(['"', display_type, '" == "ssd1306"']))

    common_params = {
        'display_type': display_type,
        'i2c_bus': i2c_bus,
        'scroll_period': scroll_period,
    }

    return LaunchDescription([
        DeclareLaunchArgument('display_type', default_value='lcm1602',
                              choices=['lcm1602', 'ssd1306'],
                              description='Display type: lcm1602 or ssd1306.'),
        DeclareLaunchArgument('i2c_bus', default_value='1',
                              description='I2C bus number.'),
        DeclareLaunchArgument('scroll_period', default_value='1.0',
                              description='Seconds between scroll steps.'),
        DeclareLaunchArgument('device_id_lcm1602', default_value='39',
                              description='I2C address for LCM1602 (default 0x27 = 39).'),
        DeclareLaunchArgument('device_id_ssd1306', default_value='60',
                              description='I2C address for SSD1306 (default 0x3C = 60).'),
        DeclareLaunchArgument('rows', default_value='4',
                              description='LCM1602 rows.'),
        DeclareLaunchArgument('columns', default_value='20',
                              description='LCM1602 columns.'),

        Node(
            package='linux_i2c_demos',
            executable='screen_from_str_topic',
            name='screen_from_str_topic',
            output='screen',
            condition=is_lcm1602,
            parameters=[common_params, {
                'device_id': LaunchConfiguration('device_id_lcm1602'),
                'rows': rows,
                'columns': columns,
            }],
        ),
        Node(
            package='linux_i2c_demos',
            executable='screen_from_str_topic',
            name='screen_from_str_topic',
            output='screen',
            condition=is_ssd1306,
            parameters=[common_params, {
                'device_id': LaunchConfiguration('device_id_ssd1306'),
            }],
        ),
    ])
