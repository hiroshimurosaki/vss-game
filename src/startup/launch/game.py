"""O jogo completo da feira, rodando no simulador.

    ros2 launch startup game.py

Abra três coisas:

    http://localhost:8090/          TV (tela cheia, para o público)
    http://localhost:8090/operador  painel de quem toca o estande
    http://localhost:8080/          simulador (só enquanto não há robôs)

Quando os robôs estiverem prontos, este mesmo launch troca duas peças e o resto
continua igual: sai o `simulator`, entram `vision_game` (publicando /game_data)
e `radio_communication` (consumindo /motorVelocities).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    verbose = LaunchConfiguration('verbose')

    field_length = LaunchConfiguration('field_length')
    field_width = LaunchConfiguration('field_width')
    goal_width = LaunchConfiguration('goal_width')
    axle_length = LaunchConfiguration('axle_length')
    wheel_speed_max = LaunchConfiguration('wheel_speed_max')
    max_linear = LaunchConfiguration('max_linear_velocity')
    max_angular = LaunchConfiguration('max_angular_velocity')

    return LaunchDescription([
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('num_robots', default_value='2'),
        DeclareLaunchArgument('player_id', default_value='1'),
        DeclareLaunchArgument('ai_id', default_value='0'),
        DeclareLaunchArgument('difficulty', default_value='MEDIO'),

        DeclareLaunchArgument('sim_port', default_value='8080'),
        DeclareLaunchArgument('game_port', default_value='8090'),

        DeclareLaunchArgument('field_length', default_value='1.50'),
        DeclareLaunchArgument('field_width', default_value='1.30'),
        DeclareLaunchArgument('goal_width', default_value='0.40'),
        DeclareLaunchArgument('axle_length', default_value='0.0625'),
        DeclareLaunchArgument('wheel_speed_max', default_value='0.75'),
        DeclareLaunchArgument('max_linear_velocity', default_value='0.6'),
        DeclareLaunchArgument('max_angular_velocity', default_value='5.0'),

        DeclareLaunchArgument('target_score', default_value='2',
                              description='gols para vencer'),
        DeclareLaunchArgument('time_limit', default_value='180.0',
                              description='teto da partida, em segundos'),

        DeclareLaunchArgument('vision_noise', default_value='0.0'),
        DeclareLaunchArgument('vision_delay', default_value='0.0'),

        LogInfo(msg='[jogo] TV: http://localhost:8090/  |  '
                    'Operador: http://localhost:8090/operador  |  '
                    'Simulador: http://localhost:8080/'),

        Node(
            package='game_master',
            executable='master_node',
            name='game_master',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('game_port'),
                'target_score': LaunchConfiguration('target_score'),
                'time_limit': LaunchConfiguration('time_limit'),
                'field_length': field_length,
                'goal_width': goal_width,
                'difficulty': LaunchConfiguration('difficulty'),
            }],
        ),

        Node(
            package='simulator',
            executable='sim_node',
            name='simulator',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('sim_port'),
                'player_id': LaunchConfiguration('player_id'),
                'field_length': field_length,
                'field_width': field_width,
                'goal_width': goal_width,
                'axle_length': axle_length,
                'wheel_speed_max': wheel_speed_max,
                'vision_noise': LaunchConfiguration('vision_noise'),
                'vision_delay': LaunchConfiguration('vision_delay'),
                # Quem apita é o game_master.
                'auto_referee': False,
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
                'field_length': field_length,
                'field_width': field_width,
                'goal_width': goal_width,
                'max_linear_velocity': max_linear,
                'max_angular_velocity': max_angular,
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
                'max_linear_velocity': max_linear,
                'max_angular_velocity': max_angular,
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
                'axle_length': axle_length,
                'wheel_speed_max': wheel_speed_max,
                'verbose': verbose,
            }],
        ),
    ])
