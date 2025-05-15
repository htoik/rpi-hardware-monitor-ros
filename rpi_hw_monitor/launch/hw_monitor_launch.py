#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_fp = os.path.join(
        get_package_share_directory("rpi_hw_monitor"),
        "config",
        "params.yml",
    )

    return LaunchDescription(
        [
            Node(
                package="rpi_hw_monitor",
                executable="hw_monitor.py",
                name="hw_monitor",
                output="screen",
                parameters=[params_fp],
            ),
        ]
    )
