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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('linux_i2c_ros2_control')

    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'),
                ' ',
                PathJoinSubstitution([pkg_share, 'urdf', 'bmi160_demo.urdf.xacro']),
            ]
        ),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'bmi160_controllers.yaml'])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen',
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_pub,
            imu_broadcaster_spawner,
        ]
    )
