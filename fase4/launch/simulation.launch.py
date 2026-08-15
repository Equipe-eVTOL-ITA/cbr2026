#!/usr/bin/env python3
"""
Launch de SIMULACAO da missao fase4 (cbr2026).

O CAMINHO E O MESMO DAS OUTRAS FASES:

    Task "sim: iniciar com missão", com mundo e missao = cbr2026:fase4

    ou, a mao:
      bash src/cbr2026/scripts/simulate.sh fase4
      bash src/cbr2026/scripts/agent.sh
      ros2 launch fase4 simulation.launch.py

O QUE ESTA FASE ACRESCENTA, e onde isso fica escondido

Ela e a unica que usa LIDAR, e o LIDAR do Gazebo publica num topico do gz, nao
do ROS. Falta um tradutor -- e ele mora AQUI, e nao numa task propria: a forma
de rodar uma simulacao tem de ser a mesma para todas as fases, e o que difere
entre elas pertence aos arquivos delas.

    /world/<mundo>/model/<drone>/link/link/sensor/lidar_2d_v2/scan
        --> /scan   (sensor_msgs/LaserScan)

O NOME DO TOPICO E DERIVADO, e nao escolhido. Quando um sensor do gz nao declara
`<topic>`, o nome sai de mundo/modelo/link/sensor -- entao trocar o mundo ou o
modelo no simulate.sh muda o topico, e a ponte para de casar sem dizer nada. Os
dois sao argumentos deste launch por causa disso.

PARA O SIMULADOR 2D, use a task "sim2d: rodar uma fase". Ela NAO passa por aqui:
sobe o `sim2d` e a missao por `ros2 run`, sem bag nem ponte, porque nada disso
existe la.

O QUE AINDA FALTA para a fase voar no Gazebo: a cadeia de localizacao. Sem GPS o
EKF do PX4 nao tem posicao horizontal. Ate a Onda 4 existir, rode com o GPS
simulado do SITL ligado -- isso separa "a navegacao funciona contra um LIDAR de
verdade?" de "o SLAM funciona?", que hoje estao embaralhadas.
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
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase4_{stamp}')

    mundo = LaunchConfiguration('mundo')
    modelo = LaunchConfiguration('modelo')

    topico_gz = PythonExpression([
        "'/world/' + '", mundo, "' + '/model/' + '", modelo,
        "' + '/link/link/sensor/lidar_2d_v2/scan'"])

    ponte_lidar = Node(
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

    # Grava um rosbag de cada voo. Depois de uma missao que deu errado, esta e
    # a unica forma de saber o que o drone via no momento.
    #
    # NESTA FASE, o que o drone "via" e o /scan: nao ha camera, e toda decisao
    # de posicao sai dele. Um bag sem /scan responde ONDE o drone foi e nao
    # responde por que -- que e a unica pergunta que se faz depois de bater.
    #
    # E a odometria e /fmu/out/vehicle_odometry, que e o que a classe Drone
    # assina. O vehicle_local_position, que o gerador traz por padrao, nao e
    # publicado pelo simulador: o bag saia vazio nesse topico sem que nada
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

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abrir o RViz2'),
        DeclareLaunchArgument(
            'mundo', default_value='fase4',
            description='nome do mundo no gz; TEM de casar com o simulate.sh'),
        DeclareLaunchArgument(
            'modelo', default_value='wanda_0',
            description='nome da instancia do modelo no gz (o PX4 acrescenta _0)'),

        ponte_lidar,
        bag,
        system_health,
        # Dez segundos, e nao cinco: o Gazebo e o EKF do PX4 levam mais para
        # estabilizar do que os nos das fases sem simulador de fisica.
        TimerAction(period=10.0, actions=[mission]),
    ])
