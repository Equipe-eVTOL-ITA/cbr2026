#!/usr/bin/env python3
"""Launch de SIMULACAO da missao fase3 (cbr2026).

Tudo numa maquina so. A WEBCAM LOCAL faz o papel da camera do drone: o Gazebo
nao renderiza uma mao, entao a unica forma de exercitar o laco inteiro --
incluindo a latencia do classificador -- e gesticular para uma camera de
verdade enquanto o drone voa na simulacao. Foi o que 2025 fez.
"""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase3'), 'config', 'simulation.yaml')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase3_{stamp}')

    # Os topicos de gesto entram no bag porque sao o que permite REPRODUZIR uma
    # sessao depois, sem camera e sem operador -- ver replay.launch.py. Sem eles
    # o bag mostra o que o drone fez, mas nao o que ele viu.
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/rosout',
             '/drone_trajectory',
             '/telemetry/drone_status',
             '/gesture_detector/gestures',
             '/gesture_detector/hand_location',
             '/fmu/out/vehicle_local_position',
             '/fmu/out/vehicle_status',
             '/fmu/in/trajectory_setpoint'],
        output='screen')

    camera = Node(
        package='camera_publisher', executable='webcam',
        parameters=[params], output='screen')

    gesture_detector = Node(
        package='gesture_detector', executable='gesture_detector',
        parameters=[params], output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
        parameters=[params], output='screen')

    mission = Node(
        package='fase3', executable='fase3',
        parameters=[params], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abrir o RViz2'),
        bag,
        camera,
        gesture_detector,
        system_health,
        # A FSM espera 8 s. Mais que a fase 1 de proposito: o mediapipe carrega
        # um modelo de 8 MB e o primeiro quadro demora. Decolar antes disso
        # significa comecar a girar procurando uma mao que o classificador ainda
        # nao consegue ver.
        TimerAction(period=8.0, actions=[mission]),
    ])
