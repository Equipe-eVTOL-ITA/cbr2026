#!/usr/bin/env python3
"""Launch de SOLO da fase3 — roda no COMPUTADOR DE TERRA.

Recebe a imagem do drone pela rede, reconhece os gestos e roda a FSM. A camera
NAO sobe aqui: ela esta no drone, com onboard.launch.py.

Suba o onboard PRIMEIRO. Este launch espera 8 s antes de iniciar a FSM, mas se
o drone ainda nao estiver publicando imagem, o drone vai decolar e girar
procurando uma mao que ninguem esta enxergando.
"""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase3'), 'config', 'flight.yaml')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase3_voo_{stamp}')

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

    gesture_detector = Node(
        package='gesture_detector', executable='gesture_detector',
        parameters=[params], output='screen')

    mission = Node(
        package='fase3', executable='fase3',
        parameters=[params], output='screen')

    return LaunchDescription([
        bag,
        gesture_detector,
        TimerAction(period=8.0, actions=[mission]),
    ])
