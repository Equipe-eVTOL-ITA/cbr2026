#!/usr/bin/env python3
"""Launch EMBARCADO da fase3 — roda NO DRONE.

Sobe so a camera. Todo o processamento (reconhecimento de gestos e FSM) roda no
computador de solo, com ground.launch.py.

O que atravessa a rede daqui para la e a imagem comprimida. Se a missao estiver
lenta para responder, meca a banda ANTES de mexer em ganho de PID:

    ros2 topic bw /camera/image/compressed
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase3'), 'config', 'flight.yaml')

    camera = Node(
        package='camera_publisher', executable='webcam',
        parameters=[params], output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
        parameters=[params], output='screen')

    return LaunchDescription([camera, system_health])
