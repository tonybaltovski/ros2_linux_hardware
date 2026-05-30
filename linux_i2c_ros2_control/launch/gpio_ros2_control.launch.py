# Copyright 2020 Tony Baltovski
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    chip_arg = DeclareLaunchArgument(
        'chip', default_value='/dev/gpiochip0',
        description='Path to the GPIO chip character device.')
    led_line_arg = DeclareLaunchArgument(
        'led_line', default_value='17',
        description='Line offset on the chip for the LED (BCM numbering on a Pi).')
    button_line_arg = DeclareLaunchArgument(
        'button_line', default_value='27',
        description='Line offset on the chip for the button (BCM numbering on a Pi).')

    pkg = FindPackageShare('linux_i2c_ros2_control')

    robot_description = {
        'robot_description': Command([
            'xacro ',
            PathJoinSubstitution([pkg, 'urdf', 'raspi_gpio.urdf.xacro']),
            ' chip:=', LaunchConfiguration('chip'),
            ' led_line:=', LaunchConfiguration('led_line'),
            ' button_line:=', LaunchConfiguration('button_line'),
        ]),
    }

    controllers_yaml = PathJoinSubstitution([pkg, 'config', 'gpio_controllers.yaml'])

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

    spawn_gpio = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gpio_command_controller'],
        output='screen',
    )

    return LaunchDescription([
        chip_arg,
        led_line_arg,
        button_line_arg,
        control_node,
        robot_state_pub,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=control_node,
                on_exit=[],
            ),
        ),
        spawn_gpio,
    ])
