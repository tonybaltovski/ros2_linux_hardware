# Copyright 2026 Tony Baltovski
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Launch the IIO IMU publisher.

Reads from any IIO device (default: bmi160) and publishes sensor_msgs/Imu and
sensor_msgs/Temperature.  Override the IIO name or device path via launch args
to target a different IMU on the system.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    iio_name_arg = DeclareLaunchArgument(
        'iio_name',
        default_value='bmi160',
        description='IIO device name (contents of iio:deviceN/name).',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='imu_link', description='IMU frame_id.'
    )
    buffered_arg = DeclareLaunchArgument(
        'use_buffered_reads',
        default_value='false',
        description='True to use /dev/iio:deviceN + poll() (requires a configured trigger).',
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='100.0', description='Polled-mode publish rate, Hz.'
    )

    node = Node(
        package='linux_iio_devices',
        executable='iio_imu_publisher',
        name='iio_imu_publisher',
        output='screen',
        parameters=[
            {
                'iio_name': LaunchConfiguration('iio_name'),
                'frame_id': LaunchConfiguration('frame_id'),
                'use_buffered_reads': LaunchConfiguration('use_buffered_reads'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ],
    )

    return LaunchDescription(
        [iio_name_arg, frame_id_arg, buffered_arg, publish_rate_arg, node]
    )
