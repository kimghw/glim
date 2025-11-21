#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Microstrain launch
    microstrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/kimghw/microstrain_ws/install/microstrain_inertial_driver/share/microstrain_inertial_driver/launch/microstrain_launch.py'
        )
    )

    return LaunchDescription([
        microstrain_launch,
        # Ouster는 이미 다른 방식으로 실행 중이면 여기 추가 안 해도 됨
    ])
