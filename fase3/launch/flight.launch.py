#!/usr/bin/env python3
"""Launch EMBARCADO da fase3 — roda NO DRONE.

Sobe o stream de imagem para o solo. Todo o processamento (reconhecimento de
gestos e FSM) roda no computador de solo, com ground.launch.py.

A CAMERA NAO SOBE AQUI POR PADRAO, e ha um motivo de hardware:

    /oak/left/image_rect ──▶ cuVSLAM        (container Isaac ROS, fica no drone)
                         └─▶ roi_stream ──▶ /gesto_camera/compressed ──rede──▶

A MESMA OAK-D serve ao SLAM e a esta missao, e o dispositivo USB so pode ser
aberto por UM processo. Quem o abre e o `camera_jetson.launch.py` do
slam_bridge. Se voce esta voando com VIO, ele JA ESTA NO AR -- subir a camera
daqui tambem faria o segundo driver falhar ao abrir o dispositivo.

Para voar os gestos SEM o SLAM, passe `start_camera:=true` e este launch abre a
camera ele mesmo:

    ros2 launch fase3 flight.launch.py start_camera:=true

Os parametros do sensor (400P, par retificado, projetor IR desligado) moram em
`slam_bridge/config/camera_params_vslam.yaml` e sao escolhas do SLAM — nao os
duplique no flight.yaml desta missao.

O `roi_stream` apenas SE INSCREVE no topico do SLAM. Nada no caminho do cuVSLAM
e recortado ou republicado: um quadro recortado invalidaria o camera_info e
destruiria a escala metrica.

O que atravessa a rede daqui para la e so o stream recortado. Se a missao
estiver lenta para responder, meca a banda ANTES de mexer em ganho de PID:

    ros2 topic bw /gesto_camera/compressed
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase3'), 'config', 'flight.yaml')

    # FindPackageShare e uma SUBSTITUICAO, resolvida na hora do lancamento e so
    # quando a condicao passa. get_package_share_directory('slam_bridge') aqui
    # exigiria o slam_bridge instalado ate para quem nunca sobe a camera.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_bridge'),
                                  'launch', 'camera_jetson.launch.py'])),
        condition=IfCondition(LaunchConfiguration('start_camera')))

    # Recorta a ROI e reduz a taxa ANTES da rede. Ver o bloco `roi_stream:` no
    # flight.yaml para o porque de cada numero.
    roi_stream = Node(
        package='camera_publisher', executable='roi_stream',
        parameters=[params], output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
        parameters=[params], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera', default_value='false',
            description='Abrir a OAK-D aqui. Deixe false se o SLAM ja estiver '
                        'no ar: o dispositivo so aceita um processo.'),
        camera,
        roi_stream,
        system_health,
    ])
