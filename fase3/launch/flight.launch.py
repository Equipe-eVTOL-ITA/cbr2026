#!/usr/bin/env python3
"""Launch EMBARCADO da fase3 — roda NO DRONE.

Sobe so o stream da camera. Todo o processamento (reconhecimento de gestos e
FSM) roda no computador de solo, com ground.launch.py.

A fonte da imagem e o `roi_stream`, que recorta e reduz a taxa do
`/oak/left/image_rect` ANTES da rede, e publica em `/gesto_camera/compressed`.

NAO sobe o driver do OAK-D: `/oak/left/image_rect` vem da stack de VSLAM, que e
lancada a parte. Sem ela este launch sobe sem erro nenhum e nao publica quadro
algum -- confira antes de armar:

    ros2 topic hz /oak/left/image_rect
    ros2 topic hz /gesto_camera/compressed

O que atravessa a rede daqui para la e a imagem comprimida. Se a missao estiver
lenta para responder, meca a banda ANTES de mexer em ganho de PID:

    ros2 topic bw /gesto_camera/compressed
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase3'), 'config', 'flight.yaml')

    camera = Node(
        package='camera_publisher', executable='roi_stream',
        parameters=[params], output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
        parameters=[params], output='screen')

    return LaunchDescription([camera, system_health])
