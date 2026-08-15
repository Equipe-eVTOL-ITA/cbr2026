#!/usr/bin/env python3
"""Launch de SIMULACAO da missao fase4 (cbr2026)."""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase4'), 'config', 'simulation.yaml')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase4_{stamp}')

    # Grava um rosbag de cada voo. Depois de uma missao que deu errado, esta e
    # a unica forma de saber o que o drone via no momento.
    #
    # NESTA FASE, o que o drone "via" e o /scan: nao ha camera, e toda decisao
    # de posicao sai dele. Um bag sem /scan responde onde o drone foi e nao
    # responde por que -- que e a unica pergunta que se faz depois de bater.
    #
    # E a odometria e /fmu/out/vehicle_odometry, que e o que a classe Drone
    # assina e o que o sim2d publica. O vehicle_local_position, que estava aqui,
    # nao e publicado pelo simulador: o bag saia vazio nesse topico sem que nada
    # avisasse.
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/rosout',
             '/scan',
             '/drone_trajectory',
             '/telemetry/drone_status',
             '/fmu/out/vehicle_odometry',
             '/fmu/out/vehicle_status',
             '/fmu/in/trajectory_setpoint'],
        output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
        parameters=[params], output='screen')

    mission = Node(
        package='fase4', executable='fase4',
        parameters=[params], output='screen')

    # ACRESCENTE aqui os nos de visao desta missao, ex.:
    # vision = Node(package='cv_nodes_algum', executable='detector', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abrir o RViz2'),
        bag,
        system_health,
        # A FSM espera 5 s para os outros nos subirem antes de comecar.
        TimerAction(period=5.0, actions=[mission]),
    ])
