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
from launch.conditions import IfCondition, UnlessCondition
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

        # Quem publica /game_data: o simulador ou a câmera. É a troca inteira —
        # os dois publicam a mesma mensagem, então IA, game_master e TV não
        # sabem qual dos dois está do outro lado.
        DeclareLaunchArgument('use_vision', default_value='false',
                              description='true = câmera, false = simulador'),
        DeclareLaunchArgument('camera', default_value='/dev/video2'),
        DeclareLaunchArgument('vision_port', default_value='8070'),

        # Como o jogador humano entra. No simulador o teclado da GUI publica o
        # /joy dele; com a câmera esse produtor some junto com o simulador, e
        # sem uma destas duas opções o visitante fica sem volante.
        DeclareLaunchArgument('use_joy', default_value='false',
                              description='controle físico para o jogador'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('deadzone', default_value='0.10'),
        DeclareLaunchArgument('use_keyboard', default_value='false',
                              description='WASD no lugar do controle'),

        # A outra metade da troca do simulador: com a câmera, ninguém consome
        # /motorVelocities — no simulador quem consumia era o próprio sim_node.
        # Fica desligado por padrão porque, sem o Arduino na porta, o nó só
        # enche o log de erro de serial.
        DeclareLaunchArgument('use_radio', default_value='false',
                              description='sobe a ponte serial para os robôs'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('tx_rate_hz', default_value='30.0'),

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
            package='vision_game',
            executable='vision_node',
            name='vision_game',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_vision')),
            parameters=[{
                'device': LaunchConfiguration('camera'),
                'port': LaunchConfiguration('vision_port'),
            }],
            # ver comentário em vision.py: numpy 2.x do user-site quebra o cv2.
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),

        Node(
            package='simulator',
            executable='sim_node',
            name='simulator',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_vision')),
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

        # Controle físico do visitante. Remapeado para o /joy_N do robô dele —
        # o `game_controller_node` vem do pacote `joy` do ROS, não deste repo.
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_joy')),
            remappings=[('/joy', ['/joy_', LaunchConfiguration('player_id')])],
            parameters=[{
                'device_id': LaunchConfiguration('joy_device_id'),
                'deadzone': LaunchConfiguration('deadzone'),
                'autorepeat_rate': 20.0,
                'coalesce_interval_ms': 1,
            }],
        ),

        # Alternativa sem hardware. O keyboard_input publica em /joy_0 fixo,
        # que é o robô da IA — então só faz sentido com a IA desligada
        # (`ros2 topic pub --once /ai/enabled std_msgs/Bool '{data: false}'`),
        # para dirigir na mão e conferir a cinemática.
        Node(
            package='controller_interpreter',
            executable='keyboard_input',
            name='keyboard_input',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_keyboard')),
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

        Node(
            package='robot_communication',
            executable='radio_communication',
            name='radio_communication',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_radio')),
            parameters=[{
                'device_name': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'tx_rate_hz': LaunchConfiguration('tx_rate_hz'),
                'verbose': verbose,
            }],
        ),
    ])
