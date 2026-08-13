#!/usr/bin/env python3
"""Launch de VOO REAL da missao fase1 (cbr2026)."""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase1'), 'config', 'flight.yaml')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase1_{stamp}')

    # Grava um rosbag de cada voo. Depois de uma missao que deu errado, esta e
    # a unica forma de saber o que o drone via no momento.
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/rosout',
             '/drone_trajectory',
             '/telemetry/drone_status',
             '/telemetry/bases',
             '/base_detector/detections',
             '/fmu/out/vehicle_local_position',
             '/fmu/out/vehicle_status',
             '/fmu/in/trajectory_setpoint'],
        output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
        parameters=[params], output='screen')

    # Em voo a imagem vem do camera_publisher, e nao de uma ponte do Gazebo.
    # Confira o `image_topic` no flight.yaml: se ele nao casar com o topico que
    # o camera_publisher realmente publica, o detector sobe, nao reclama e
    # nunca recebe quadro -- a missao voa cega sem dizer por que.
    camera = Node(
        package='camera_publisher', executable='webcam',
        parameters=[params], output='screen')

    base_detector = Node(
        package='base_detector', executable='base_detector',
        parameters=[params], output='screen')

    mission = Node(
        package='fase1', executable='fase1',
        parameters=[params], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abrir o RViz2'),
        bag,
        camera,
        system_health,
        base_detector,
        # A FSM espera 5 s para os outros nos subirem antes de comecar. Sem
        # isso ela decola antes de o detector estar pronto e varre o primeiro
        # trecho da grade cega.
        TimerAction(period=5.0, actions=[mission]),
    ])
