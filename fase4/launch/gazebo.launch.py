#!/usr/bin/env python3
"""
Fase 4 no Gazebo: a ponte do LIDAR mais a missao.

ESTE LAUNCH NAO SOBE O GAZEBO. O PX4 e quem o sobe, pelo scripts/simulate.sh --
e por isso a ordem dos terminais importa:

    1) bash src/cbr2026/scripts/simulate.sh fase4
    2) bash src/cbr2026/scripts/agent.sh
    3) ros2 launch fase4 gazebo.launch.py

O QUE ELE ACRESCENTA AO simulation.launch.py

Uma ponte. O LIDAR do Gazebo publica num topico do gz, e a missao assina `/scan`
do ROS; sem tradutor no meio, os dois nao se falam e a missao espera scan para
sempre, sem erro nenhum.

    /world/<mundo>/model/<drone>/link/link/sensor/lidar_2d_v2/scan
        --> /scan   (sensor_msgs/LaserScan)

O NOME DO TOPICO E DERIVADO, e nao escolhido. Quando um sensor do gz nao declara
`<topic>`, o nome sai de mundo/modelo/link/sensor -- entao trocar o mundo ou o
modelo no simulate.sh muda o topico, e a ponte para de casar sem dizer nada. Os
dois sao argumentos deste launch por causa disso.

O QUE AINDA NAO ESTA AQUI

A cadeia de localizacao. Sem GPS o EKF do PX4 nao tem posicao horizontal, e o
`getLocalPosition()` da missao devolve lixo. A cadeia
`slam_toolbox -> adaptador -> slam_bridge` e a Onda 4 e nao existe.

Enquanto ela nao existe, RODE COM O GPS SIMULADO DO SITL LIGADO (que e o
padrao). Isso separa duas perguntas que hoje estao embaralhadas:

  - a navegacao, a arvore e o `scan_fit` funcionam contra um LIDAR de verdade,
    com ruido de verdade e 270 graus de campo em vez dos meus 360 perfeitos?
  - a localizacao por SLAM funciona?

A primeira da para responder agora. A segunda nao, e misturar as duas faz uma
falha de qualquer uma parecer falha da outra.
"""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase4'), 'config',
                          'simulation.yaml')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase4_gz_{stamp}')

    mundo = LaunchConfiguration('mundo')
    modelo = LaunchConfiguration('modelo')

    # O topico do gz, montado a partir do mundo e do modelo. `PythonExpression`
    # porque as duas partes so existem em tempo de lancamento.
    topico_gz = PythonExpression([
        "'/world/' + '", mundo, "' + '/model/' + '", modelo,
        "' + '/link/link/sensor/lidar_2d_v2/scan'"])

    ponte = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='ponte_lidar',
        arguments=[
            PythonExpression(["'", topico_gz,
                              "' + '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'"]),
        ],
        # O topico do gz e longo e derivado; a missao assina `/scan` e nao
        # precisa saber disso.
        remappings=[(topico_gz, '/scan')],
        output='screen')

    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/rosout', '/scan', '/drone_trajectory',
             '/telemetry/drone_status',
             '/fmu/out/vehicle_odometry', '/fmu/out/vehicle_status',
             '/fmu/in/trajectory_setpoint'],
        output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
        parameters=[params], output='screen')

    mission = Node(
        package='fase4', executable='fase4',
        parameters=[params], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mundo', default_value='fase4',
            description='nome do mundo no gz; TEM de casar com o simulate.sh'),
        DeclareLaunchArgument(
            'modelo', default_value='x500_lidar_2d_0',
            description='nome da instancia do modelo no gz (o PX4 acrescenta _0)'),

        ponte,
        bag,
        system_health,
        # Mais tempo que no sim2d: o Gazebo e o EKF do PX4 levam mais para
        # estabilizar do que dois processos Python.
        TimerAction(period=10.0, actions=[mission]),
    ])
