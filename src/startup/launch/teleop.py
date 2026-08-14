"""Sobe o pipeline de teleoperação: controle -> cinemática -> rádio.

É a base sobre a qual o jogo é montado. Rodando isto, cada controle plugado
dirige um robô. O nó da IA, quando entrar, publica um /joy_N sintético e passa
por este mesmo caminho.

Uso:
    ros2 launch startup teleop.py
    ros2 launch startup teleop.py num_robots:=2 verbose:=true
    ros2 launch startup teleop.py use_keyboard:=true num_robots:=1

    # os dois robôs na mão: teclado no 0, gamepad no 1
    ros2 launch startup teleop.py num_robots:=2 use_keyboard:=true \\
        keyboard_id:=0 serial_port:=/dev/ttyUSB0

Cada controle precisa do seu próprio game_controller_node publicando em /joy_N.
Este launch cria um para cada robô que o teclado não ocupa, com device_id
contado na ordem dos robôs (o primeiro gamepad plugado pega a primeira vaga
livre). Para descobrir qual device é qual:
    ls /dev/input/js*

A porta do rádio é a da PONTE, e o número dela não é estável — com o robô
também plugado no USB, o /dev/ttyUSB0 pode ser qualquer uma das duas. Confira
com `ls /dev/ttyUSB*` e passe `serial_port:=` explícito.
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
    keyboard_id = int(LaunchConfiguration('keyboard_id').perform(context))
    verbose = _bool(context, 'verbose')

    log_level = 'info' if verbose else 'warn'

    nodes = []
    avisos = []

    # O joy_aggregator só assina /joy_0../joy_{num_robots-1}: um teclado fora
    # dessa faixa publica para ninguém, e o sintoma é teclado mudo com todo o
    # resto do stack aparentemente saudável.
    if use_keyboard and not 0 <= keyboard_id < num_robots:
        avisos.append(LogInfo(
            msg=f'[startup] AVISO: keyboard_id={keyboard_id} está fora de '
                f'0..{num_robots - 1}; o joy_aggregator não vai escutar esse '
                f'/joy_{keyboard_id}. Usando 0.'))
        keyboard_id = 0

    # O teclado ocupa a vaga do keyboard_id e os gamepads ficam com o resto.
    # Antes o teclado era fixo no robô 0 e os gamepads começavam no 1; agora
    # que o KeyboardInput tem parâmetro, qualquer combinação vale — o que
    # importa é que ninguém publique duas vezes no mesmo /joy_N, porque as duas
    # fontes se atropelam a 20-60 Hz e o robô fica gaguejando.
    joy_ids = [i for i in range(num_robots) if not (use_keyboard and i == keyboard_id)]

    if use_keyboard:
        nodes.append(Node(
            package='controller_interpreter',
            executable='keyboard_input',
            name='keyboard_input',
            output='screen',
            parameters=[{'robot_id': keyboard_id}],
        ))

    # device_id conta na ordem das vagas livres: com o teclado no robô 0, o
    # primeiro gamepad que o SDL enxerga (js0) vira o robô 1.
    for device_id, robot_id in enumerate(joy_ids):
        nodes.append(Node(
            package='joy',
            executable='game_controller_node',
            name=f'game_controller_node_{robot_id}',
            output='screen',
            remappings=[('/joy', f'/joy_{robot_id}')],
            parameters=[{
                'device_id': device_id,
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

    quem = ([f'robô {keyboard_id}: teclado'] if use_keyboard else []) \
        + [f'robô {r}: gamepad js{d}' for d, r in enumerate(joy_ids)]

    return [
        *avisos,
        LogInfo(msg=f'[startup] teleop | robos: {num_robots} | ' + ' | '.join(quem)),
        *nodes,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2'),
        DeclareLaunchArgument('use_keyboard', default_value='false'),
        DeclareLaunchArgument('keyboard_id', default_value='0',
                              description='qual robô o teclado dirige; os '
                                          'gamepads pegam as vagas restantes'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('deadzone', default_value='0.10'),
        DeclareLaunchArgument('max_linear_velocity', default_value='0.6'),
        DeclareLaunchArgument('max_angular_velocity', default_value='5.0'),
        DeclareLaunchArgument('invert_direction', default_value='false'),
        DeclareLaunchArgument('trigger_mode', default_value='sdl',
                              description='sdl (solto=0, fundo=-1; o padrão do repo) '
                                          'ou unit (fundo=+1) ou signed (solto=+1, joy_node clássico)'),
        DeclareLaunchArgument('axle_length', default_value='0.0625'),
        DeclareLaunchArgument('wheel_speed_max', default_value='0.75'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('tx_rate_hz', default_value='60.0'),
        OpaqueFunction(function=_setup),
    ])
