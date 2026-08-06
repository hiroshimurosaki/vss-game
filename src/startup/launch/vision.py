"""Só a visão, para calibrar.

    ros2 launch startup vision.py

Abra http://localhost:8070, clique os 4 cantos do campo e ajuste as cores.
Salvar grava em ~/.vss-game/vision.json, e daí em diante o `game.py` com
use_vision:=true usa a mesma calibração.

Para conferir o que está saindo, sem GUI nenhuma:

    ros2 topic echo /game_data --once
    ros2 topic hz /game_data
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/video2'),
        DeclareLaunchArgument('port', default_value='8070'),
        DeclareLaunchArgument('backend', default_value='ffmpeg',
                              description='ffmpeg (2x mais rápido) ou opencv'),

        LogInfo(msg='[visão] calibração: http://localhost:8070'),

        Node(
            package='vision_game',
            executable='vision_node',
            name='vision_game',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'port': LaunchConfiguration('port'),
                'backend': LaunchConfiguration('backend'),
            }],
            # O cv2 do apt é compilado contra numpy 1.x. Se houver um numpy 2.x
            # em ~/.local — e neste micro há — ele sombreia o do sistema e o
            # `import cv2` morre com "numpy.core.multiarray failed to import".
            # Ignorar o user-site resolve sem mexer nos pacotes do usuário.
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),
    ])
