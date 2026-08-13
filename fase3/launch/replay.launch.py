#!/usr/bin/env python3
"""Launch de REPLAY da fase3 — reproduz uma sessao gravada.

Sobe SO a missao. Os gestos vem de um bag, nao de uma camera:

    ros2 bag play <bag> --topics /gesture_detector/gestures \\
                                 /gesture_detector/hand_location

Serve para dois casos:

  - REGRESSAO: a mesma sessao deve produzir a mesma sequencia de estados. E o
    unico jeito de comparar duas versoes do codigo sem depender de um operador
    gesticular igual duas vezes;
  - DEPURACAO: reproduzir um voo que deu errado, sem hardware e quantas vezes
    for preciso.

O drone continua sendo necessario -- via SITL ou de verdade -- porque a FSM
comanda de fato. Para so conferir a leitura de gestos, sem voar, olhe os topicos
com `ros2 topic echo`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perfil = LaunchConfiguration('perfil')

    def mission(context):
        nome = perfil.perform(context)
        params = os.path.join(
            get_package_share_directory('fase3'), 'config', f'{nome}.yaml')
        return [Node(package='fase3', executable='fase3',
                     parameters=[params], output='screen')]

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        DeclareLaunchArgument(
            'perfil', default_value='simulation',
            description='qual config usar: simulation ou flight'),
        OpaqueFunction(function=mission),
    ])
