#!/usr/bin/env python3
"""Launch de SIMULACAO da missao fase1 (cbr2026)."""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase1'), 'config', 'simulation.yaml')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase1_{stamp}')

    # Grava um rosbag de cada voo. Depois de uma missao que deu errado, esta e
    # a unica forma de saber o que o drone via no momento.
    #
    # /base_detector/detections entra na lista porque e o que separa "a visao
    # nao viu a base" de "a visao viu e a projecao errou o lugar" -- duas
    # falhas que parecem identicas olhando so a trajetoria.
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

    # Ponte de imagem do Gazebo para o ROS.
    #
    # O `image_bridge` do ros_gz_image publica <topico> e <topico>/compressed;
    # e o segundo que o base_detector assina, porque a classe Detector trabalha
    # com CompressedImage. Sem esta ponte o detector sobe, nao reclama de nada
    # e simplesmente nunca recebe quadro.
    #
    # Requer ros-humble-ros-gzgarden-image, que o env/<perfil>.yaml ja exige.
    # A variante ros-gz-image (Fortress) instala mas nao enxerga topico algum
    # do Gazebo Garden -- e sem mensagem de erro. Ver a lista de pacotes
    # proibidos no perfil.
    image_bridge = Node(
        package='ros_gz_image', executable='image_bridge',
        arguments=['/vertical_camera'],
        output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
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
        image_bridge,
        system_health,
        base_detector,
        # A FSM espera 5 s para os outros nos subirem antes de comecar. Sem
        # isso ela decola antes de o detector estar pronto e varre o primeiro
        # trecho da grade cega.
        TimerAction(period=5.0, actions=[mission]),
    ])
