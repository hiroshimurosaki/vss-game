"""Sobe o pipeline de teleoperação: controle -> cinemática -> rádio.

É a base sobre a qual o jogo é montado. Rodando isto, cada controle plugado
dirige um robô. O nó da IA, quando entrar, publica um /joy_N sintético e passa
por este mesmo caminho.

Uso:
    ros2 launch startup teleop.py
    ros2 launch startup teleop.py num_robots:=2 verbose:=true
    ros2 launch startup teleop.py use_keyboard:=true num_robots:=1

Cada controle precisa do seu próprio game_controller_node publicando em /joy_N.
Este launch cria um por robô, com device_id igual ao índice. Para descobrir qual
device é qual:
    ls /dev/input/js*
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _setup(context):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    use_keyboard = _bool(context, 'use_keyboard')
    verbose = _bool(context, 'verbose')

    log_level = 'info' if verbose else 'warn'

    nodes = []

    # O KeyboardInput publica em /joy_0, então ele sempre ocupa o robô 0 e os
    # controles físicos começam no 1.
    first_joy_id = 1 if use_keyboard else 0

    if use_keyboard:
        nodes.append(Node(
            package='controller_interpreter',
            executable='keyboard_input',
            name='keyboard_input',
            output='screen',
        ))

    for robot_id in range(first_joy_id, num_robots):
        nodes.append(Node(
            package='joy',
            executable='game_controller_node',
            name=f'game_controller_node_{robot_id}',
            output='screen',
            remappings=[('/joy', f'/joy_{robot_id}')],
            parameters=[{
                'device_id': robot_id - first_joy_id,
                'deadzone': LaunchConfiguration('deadzone'),
                'autorepeat_rate': 20.0,
                'coalesce_interval_ms': 1,
            }],
            arguments=['--ros-args', '--log-level', log_level],
        ))

    nodes += [
        Node(
            package='controller_interpreter',
            executable='joy_aggregator',
            name='joy_aggregator',
            output='screen',
            parameters=[{'num_robots': num_robots}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='controller_interpreter',
            executable='direction',
            name='direction',
            output='screen',
            parameters=[{
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'invert_direction': LaunchConfiguration('invert_direction'),
                'trigger_mode': LaunchConfiguration('trigger_mode'),
                'verbose': verbose,
            }],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='controller_interpreter',
            executable='special_controls',
            name='special_controls',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
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
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='robot_communication',
            executable='radio_communication',
            name='radio_communication',
            output='screen',
            parameters=[{
                'device_name': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'tx_rate_hz': LaunchConfiguration('tx_rate_hz'),
                'verbose': verbose,
            }],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ]

    return [
        LogInfo(msg=f'[startup] teleop | robos: {num_robots} | teclado: {use_keyboard}'),
        *nodes,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2'),
        DeclareLaunchArgument('use_keyboard', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('deadzone', default_value='0.10'),
        DeclareLaunchArgument('max_linear_velocity', default_value='0.6'),
        DeclareLaunchArgument('max_angular_velocity', default_value='5.0'),
        DeclareLaunchArgument('invert_direction', default_value='false'),
        DeclareLaunchArgument('trigger_mode', default_value='signed',
                              description='signed (solto=+1) ou unit (solto=0)'),
        DeclareLaunchArgument('axle_length', default_value='0.0625'),
        DeclareLaunchArgument('wheel_speed_max', default_value='0.75'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('tx_rate_hz', default_value='60.0'),
        OpaqueFunction(function=_setup),
    ])
