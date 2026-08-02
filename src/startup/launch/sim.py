"""Sobe o simulador com o pipeline completo — sem hardware nenhum.

É o mesmo caminho de código que roda com os robôs de verdade. A única diferença
é quem está nas pontas:

    com hardware:   game_controller_node -> ... -> radio_communication -> robô
    no simulador:   teclado da GUI       -> ... -> sim_node (física)

Convenção do jogo: robô 0 é a IA (defende a esquerda), robô 1 é o visitante.

Uso:
    ros2 launch startup sim.py
    ros2 launch startup sim.py vision_noise:=0.005 vision_delay:=0.08

Depois abra http://localhost:8080
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    verbose = LaunchConfiguration('verbose')

    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2'),
        DeclareLaunchArgument('player_id', default_value='1'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('port', default_value='8080'),

        DeclareLaunchArgument('max_linear_velocity', default_value='0.6'),
        DeclareLaunchArgument('max_angular_velocity', default_value='5.0'),
        DeclareLaunchArgument('axle_length', default_value='0.0625'),
        DeclareLaunchArgument('wheel_speed_max', default_value='0.75'),

        DeclareLaunchArgument('field_length', default_value='1.50'),
        DeclareLaunchArgument('field_width', default_value='1.30'),
        DeclareLaunchArgument('goal_width', default_value='0.40'),

        # Imperfeições da visão. Deixe em zero para desenvolver a IA e ligue
        # depois, para conferir se ela aguenta o que a câmera real vai entregar.
        DeclareLaunchArgument('vision_noise', default_value='0.0',
                              description='ruído nas posições publicadas, em metros'),
        DeclareLaunchArgument('vision_delay', default_value='0.0',
                              description='atraso de /game_data, em segundos'),

        DeclareLaunchArgument('ai_id', default_value='0'),
        DeclareLaunchArgument('difficulty', default_value='MEDIO',
                              description='FACIL | MEDIO | DIFICIL'),

        LogInfo(msg='[sim] abra http://localhost:8080 — WASD dirige o robô 1'),

        Node(
            package='simulator',
            executable='sim_node',
            name='simulator',
            output='screen',
            parameters=[{
                'player_id': LaunchConfiguration('player_id'),
                'port': LaunchConfiguration('port'),
                'field_length': LaunchConfiguration('field_length'),
                'field_width': LaunchConfiguration('field_width'),
                'goal_width': LaunchConfiguration('goal_width'),
                'axle_length': LaunchConfiguration('axle_length'),
                'wheel_speed_max': LaunchConfiguration('wheel_speed_max'),
                'vision_noise': LaunchConfiguration('vision_noise'),
                'vision_delay': LaunchConfiguration('vision_delay'),
            }],
        ),

        Node(
            package='ai_player',
            executable='ai_node',
            name='ai_player',
            output='screen',
            parameters=[{
                'robot_id': LaunchConfiguration('ai_id'),
                'difficulty': LaunchConfiguration('difficulty'),
                'field_length': LaunchConfiguration('field_length'),
                'field_width': LaunchConfiguration('field_width'),
                'goal_width': LaunchConfiguration('goal_width'),
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            }],
        ),

        Node(
            package='controller_interpreter',
            executable='joy_aggregator',
            name='joy_aggregator',
            output='screen',
            parameters=[{'num_robots': LaunchConfiguration('num_robots')}],
        ),

        Node(
            package='controller_interpreter',
            executable='direction',
            name='direction',
            output='screen',
            parameters=[{
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'trigger_mode': 'signed',
                'verbose': verbose,
            }],
        ),

        Node(
            package='controller_interpreter',
            executable='special_controls',
            name='special_controls',
            output='screen',
        ),

        Node(
            package='cinematica',
            executable='kinematics_node',
            name='cinematica',
            output='screen',
            parameters=[{
                'axle_length': LaunchConfiguration('axle_length'),
                'wheel_speed_max': LaunchConfiguration('wheel_speed_max'),
                'verbose': verbose,
            }],
        ),
    ])
