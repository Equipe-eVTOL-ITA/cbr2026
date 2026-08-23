#!/usr/bin/env python3
"""Launch de SOLO da fase1 — roda no COMPUTADOR DE TERRA.

Nao sobe nada que voe: assina por DDS o que o drone publica e mostra. O drone
roda o flight.launch.py, que continua enxuto.

Existe porque o perfil `jetson-humble` nao tem `ros-humble-desktop` -- nao ha
rviz2 no drone, e nem deveria: GUI ali e CPU tirada do controle.
"""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase1'), 'config', 'flight.yaml')
    rviz_cfg = os.path.join(get_package_share_directory('fase1'), 'rviz', 'fase1.rviz')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase1_solo_{stamp}')

    # Um bag do lado de TERRA, alem do que o drone grava. Se o enlace cair no
    # meio do voo, este mostra ate onde a estacao enxergou -- e comparar os
    # dois separa "o drone parou" de "a radio parou".
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/rosout',
             '/drone_trajectory',
             '/telemetry/position',
             '/telemetry/logs',
             '/telemetry/drone_status',
             '/telemetry/system_health',
             '/telemetry/bases',
             '/base_detector/detections'],
        output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    # Converte a telemetria nos topicos que o RViz entende: as bases viram
    # marcadores, a posicao vira pose e trajetoria.
    telemetry = Node(
        package='telemetry_handler', executable='telemetry_handler',
        parameters=[params], output='screen')

    # Painel de status: bateria, modo de voo, saude do computador, logs da FSM
    # e qualidade do enlace. Confira o `ping_target` no flight.yaml.
    dashboard = Node(
        package='telemetry_handler', executable='telemetry_dashboard',
        parameters=[params], output='screen',
        condition=IfCondition(LaunchConfiguration('dashboard')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abrir o RViz2'),
        DeclareLaunchArgument('dashboard', default_value='true',
                              description='Abrir o painel de status (Tk)'),
        bag,
        telemetry,
        rviz,
        dashboard,
    ])
