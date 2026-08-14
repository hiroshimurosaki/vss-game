"""O duelo de revezamento: UM robô só, dois motoristas, turnos alternados.

    ros2 launch startup duelo.py                    # no simulador
    ros2 launch startup duelo.py use_vision:=true use_radio:=true use_joy:=true \\
        serial_port:=$(./tools/porta.sh)            # na feira, com o robô de verdade

Abra:

    http://localhost:8090/          TV (tela cheia, para o público)
    http://localhost:8090/operador  painel de quem toca o estande
    http://localhost:8080/          simulador (só enquanto não há robô)

## Para que serve

Quando só há um robô funcionando, o formato normal (dois robôs no campo ao mesmo
tempo) simplesmente não acontece. Este launch troca o duelo simultâneo por um
duelo alternado: o visitante joga o turno dele, o Franky joga o turno dele NO
MESMO ROBÔ, e ganha o round quem levou menos tempo até o gol. Melhor de três.

## As duas diferenças que importam em relação ao game.py

**1. Um robô, e ele é o 0.** A convenção do projeto não muda — o robô 0 é o da
IA — o visitante é que toma o volante emprestado nos turnos dele. Grave o robô
com `./tools/gravar.sh feira --id 0`, e é a etiqueta AMARELA (team_a) que a
visão precisa enxergar.

**2. Ninguém publica direto no /joy_0.** As duas fontes vão para tópicos
privados (`/duelo/joy_humano` e `/duelo/joy_ia`) e o `turn_mux` é o único
publicador do tópico que o robô ouve. É o que impede o acidente clássico deste
projeto: dois produtores no mesmo /joy_N fazem o robô gaguejar sem NADA no log
explicando. Ver a docstring do turn_mux.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    verbose = LaunchConfiguration('verbose')

    field_length = LaunchConfiguration('field_length')
    field_width = LaunchConfiguration('field_width')
    goal_width = LaunchConfiguration('goal_width')
    axle_length = LaunchConfiguration('axle_length')
    wheel_speed_max = LaunchConfiguration('wheel_speed_max')
    max_linear = LaunchConfiguration('max_linear_velocity')
    max_angular = LaunchConfiguration('max_angular_velocity')

    robot_id = LaunchConfiguration('robot_id')
    home_x = LaunchConfiguration('home_x')
    home_y = LaunchConfiguration('home_y')

    # O tópico que o robô realmente ouve. Só o turn_mux publica nele.
    joy_do_robo = ['/joy_', robot_id]

    # Ligado se QUALQUER volante externo estiver plugado — nesse caso o teclado
    # da GUI do simulador se cala, senão os zeros que ele publica a 60 Hz
    # atropelam o comando de verdade.
    volante_externo = PythonExpression([
        "'", LaunchConfiguration('use_joy'), "'.lower() in ('true','1') or ",
        "'", LaunchConfiguration('use_keyboard'), "'.lower() in ('true','1')",
    ])

    return LaunchDescription([
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('robot_id', default_value='0',
                              description='o único robô em campo; 0 é o da IA, '
                                          'que o visitante toma emprestado'),
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

        # ── O formato ────────────────────────────────────────────────────
        DeclareLaunchArgument('turn_limit', default_value='30.0',
                              description='teto de um turno, em segundos'),
        DeclareLaunchArgument('rounds_to_win', default_value='2',
                              description='2 = melhor de três'),
        DeclareLaunchArgument('max_rounds', default_value='5'),
        DeclareLaunchArgument('prep_min', default_value='3.0'),
        DeclareLaunchArgument('prep_max', default_value='20.0'),

        # A marca de onde todo turno começa. -99 = calcula do campo, e é a
        # mesma conta nos dois nós. Se divergirem, a IA para num ponto que o
        # árbitro não aceita e o preparo sempre vai até o teto.
        DeclareLaunchArgument('home_x', default_value='-99.0'),
        DeclareLaunchArgument('home_y', default_value='0.0'),

        DeclareLaunchArgument('vision_noise', default_value='0.0'),
        DeclareLaunchArgument('vision_delay', default_value='0.0'),

        DeclareLaunchArgument('use_vision', default_value='false',
                              description='true = câmera, false = simulador'),
        DeclareLaunchArgument('camera', default_value='/dev/video2'),
        DeclareLaunchArgument('vision_port', default_value='8070'),

        # Para quando JÁ existe uma visão no ar — a que ficou aberta na
        # calibração, por exemplo. Sem isto o launch sobe um segundo
        # vision_node, e aí há dois publicadores no /game_data: as posições
        # passam a alternar entre duas fontes e a taxa dobra, um sintoma que
        # não aponta para a causa. O aviso da porta 8070 ocupada é a única
        # pista que aparece, e ela some no meio do log.
        DeclareLaunchArgument('spawn_vision', default_value='true',
                              description='false = consome o /game_data de uma '
                                          'visão que já está rodando'),

        DeclareLaunchArgument('use_joy', default_value='false',
                              description='controle físico para o visitante'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('joy_device_name', default_value=''),
        DeclareLaunchArgument('deadzone', default_value='0.10'),
        DeclareLaunchArgument('use_keyboard', default_value='false',
                              description='WASD no lugar do controle'),

        DeclareLaunchArgument('use_radio', default_value='false',
                              description='sobe a ponte serial para o robô'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('tx_rate_hz', default_value='30.0'),

        LogInfo(msg='[duelo] TV: http://localhost:8090/  |  '
                    'Operador: http://localhost:8090/operador  |  '
                    'Simulador: http://localhost:8080/'),
        LogInfo(msg=['[duelo] um robô só (id ', robot_id, '), turnos '
                     'alternados. O volante troca pelo /game/joy_source.']),

        Node(
            package='game_master',
            executable='master_node',
            name='game_master',
            output='screen',
            parameters=[{
                'mode': 'duelo',
                'port': LaunchConfiguration('game_port'),
                'robot_id': robot_id,
                'turn_limit': LaunchConfiguration('turn_limit'),
                'rounds_to_win': LaunchConfiguration('rounds_to_win'),
                'max_rounds': LaunchConfiguration('max_rounds'),
                'prep_min': LaunchConfiguration('prep_min'),
                'prep_max': LaunchConfiguration('prep_max'),
                'home_x': home_x,
                'home_y': home_y,
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
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('use_vision'), "'.lower() in ('true','1') and ",
                "'", LaunchConfiguration('spawn_vision'), "'.lower() in ('true','1')",
            ])),
            parameters=[{
                'device': LaunchConfiguration('camera'),
                'port': LaunchConfiguration('vision_port'),
            }],
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),

        # Um robô só também no simulador: o segundo ficaria parado bem na frente
        # do gol que os dois motoristas atacam, e o teste mediria outra coisa.
        # O teclado da GUI entra como o volante do visitante — é o que permite
        # ensaiar o duelo inteiro sem gamepad e sem robô.
        Node(
            package='simulator',
            executable='sim_node',
            name='simulator',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_vision')),
            remappings=[(joy_do_robo, '/duelo/joy_humano')],
            parameters=[{
                'port': LaunchConfiguration('sim_port'),
                'num_robots': 1,
                'player_id': robot_id,
                'field_length': field_length,
                'field_width': field_width,
                'goal_width': goal_width,
                'axle_length': axle_length,
                'wheel_speed_max': wheel_speed_max,
                'vision_noise': LaunchConfiguration('vision_noise'),
                'vision_delay': LaunchConfiguration('vision_delay'),
                'auto_referee': False,
                'publish_joy': ParameterValue(
                    PythonExpression(['not (', volante_externo, ')']),
                    value_type=bool),
            }],
        ),

        # A IA dirige o MESMO robô do visitante. O /ai/home manda ela levá-lo de
        # volta à marca entre os turnos, inclusive antes do turno do visitante —
        # é o que tira a mão humana de dentro do campo.
        Node(
            package='ai_player',
            executable='ai_node',
            name='ai_player',
            output='screen',
            remappings=[(joy_do_robo, '/duelo/joy_ia')],
            parameters=[{
                'robot_id': robot_id,
                # Os presets do jogo não sobrevivem à travessia para o duelo:
                # sem adversário em campo, o FACIL fica parado no próprio gol e
                # nunca conclui (medido, 0% em 45 s). Ver PRESETS_DUELO.
                'preset_set': 'duelo',
                'difficulty': LaunchConfiguration('difficulty'),
                'home_x': home_x,
                'home_y': home_y,
                'field_length': field_length,
                'field_width': field_width,
                'goal_width': goal_width,
                'max_linear_velocity': max_linear,
                'max_angular_velocity': max_angular,
            }],
        ),

        # O revezamento do volante. ÚNICO publicador do /joy_<robot_id>.
        Node(
            package='game_master',
            executable='turn_mux',
            name='turn_mux',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'verbose': verbose,
            }],
        ),

        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_joy')),
            remappings=[('/joy', '/duelo/joy_humano')],
            parameters=[{
                'device_id': LaunchConfiguration('joy_device_id'),
                'device_name': LaunchConfiguration('joy_device_name'),
                'deadzone': LaunchConfiguration('deadzone'),
                'autorepeat_rate': 20.0,
                'coalesce_interval_ms': 1,
            }],
        ),

        Node(
            package='controller_interpreter',
            executable='keyboard_input',
            name='keyboard_input',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_keyboard')),
            remappings=[(joy_do_robo, '/duelo/joy_humano')],
            parameters=[{'robot_id': robot_id}],
        ),

        Node(
            package='controller_interpreter',
            executable='joy_aggregator',
            name='joy_aggregator',
            output='screen',
            parameters=[{'num_robots': 1}],
        ),

        Node(
            package='controller_interpreter',
            executable='direction',
            name='direction',
            output='screen',
            parameters=[{
                'max_linear_velocity': max_linear,
                'max_angular_velocity': max_angular,
                'trigger_mode': 'sdl',
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
