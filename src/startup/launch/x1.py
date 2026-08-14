"""O X1: duas PESSOAS, dois robôs, dois controles, melhor de três.

    ros2 launch startup x1.py
    ros2 launch startup x1.py serial_port:=$(./tools/porta.sh)

Abra:

    http://localhost:8090/          TV (tela cheia, para o público)
    http://localhost:8090/operador  painel de quem toca o estande

## Para que serve

É o modo que roda com o campo CEGO. Sem câmera não há /game_data, sem
/game_data não há IA — mas os dois robôs continuam funcionando pelo rádio, e
duas pessoas com dois controles é um jogo inteiro que não depende de enxergar
nada. Quem apita é gente: o gol entra pelo botão do painel do operador. Quando
a visão morre no meio da feira, este launch é o que mantém a fila andando.

As regras (round acaba no primeiro gol, melhor de três, o tempo que vale é o
melhor round vencido) moram em `game_master/x1.py`, e a bancada
`tools/x1_bench.py` roda partidas inteiras sem ROS para conferi-las.

## O que este launch NÃO sobe, e por quê

**`vision_game`** — o modo existe justamente para o caso em que não há câmera.
Subir a visão aqui seria subir o nó que a gente está assumindo que não
funciona; e se ela funcionar, o `game.py` é o launch certo.

**`ai_player`** — a IA decide a partir da posição da bola e do robô, que só
vêm da visão. Sem /game_data ela publicaria comando parado (ou lixo) no /joy_N
de alguém. Aqui os dois volantes são humanos: não há vaga para ela.

**`turn_mux`** — ele existe para revezar DUAS fontes disputando UM robô, que é
a forma do duelo. No X1 cada pessoa tem o seu robô, então cada /joy_N tem um
único produtor e o multiplexador não teria o que multiplexar. Pôr um mux com
uma fonte só é a mesma coisa que não pôr, com um nó a mais para depurar.

## Um produtor por /joy_N

Vale a mesma regra do resto do projeto: dois publicadores no mesmo /joy_N fazem
o robô gaguejar sem NADA no log explicando por quê. Aqui cada
`game_controller_node` fala com um /joy_N distinto (0 e 1) e ninguém mais
publica neles.

O device_id conta na ordem em que o SDL enxerga os controles: o primeiro
gamepad plugado (js0) vira o robô 0. Para conferir qual é qual:

    ls /dev/input/js*

A porta do rádio é a da PONTE, e o número dela não é estável — com um robô
também plugado no USB, o /dev/ttyUSB0 pode ser qualquer uma das duas. Confira
com `ls /dev/ttyUSB*` e passe `serial_port:=` explícito.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# São duas pessoas: o modo não existe com outro número. Fica como constante em
# vez de argumento de launch porque um `num_robots:=3` aqui não produziria um
# X1 maior, produziria um robô sem dono e um placar que não fecha.
NUM_ROBOTS = 2


def generate_launch_description():
    verbose = LaunchConfiguration('verbose')

    field_length = LaunchConfiguration('field_length')
    goal_width = LaunchConfiguration('goal_width')
    axle_length = LaunchConfiguration('axle_length')
    wheel_speed_max = LaunchConfiguration('wheel_speed_max')
    max_linear = LaunchConfiguration('max_linear_velocity')
    max_angular = LaunchConfiguration('max_angular_velocity')

    # Um game_controller_node por robô, cada um no seu /joy_N. Mesmo formato do
    # teleop.py, inclusive o remap e o autorepeat: 20 Hz de repetição mantém o
    # Joy vivo com o jogador imóvel, que é como o watchdog do joy_aggregator
    # distingue "parado" de "controle sumiu".
    controles = [
        Node(
            package='joy',
            executable='game_controller_node',
            name=f'game_controller_node_{robot_id}',
            output='screen',
            remappings=[('/joy', f'/joy_{robot_id}')],
            parameters=[{
                'device_id': robot_id,
                'deadzone': LaunchConfiguration('deadzone'),
                'autorepeat_rate': 20.0,
                'coalesce_interval_ms': 1,
            }],
        )
        for robot_id in range(NUM_ROBOTS)
    ]

    return LaunchDescription([
        DeclareLaunchArgument('verbose', default_value='false'),

        # ── O árbitro ────────────────────────────────────────────────────
        # Os nomes são os que o master_node declara: o que sai daqui entra lá
        # sem tradução, e parâmetro não declarado derruba o nó na hora de subir.
        DeclareLaunchArgument('port', default_value='8090',
                              description='TV e painel do operador'),

        # ── O formato ────────────────────────────────────────────────────
        DeclareLaunchArgument('rounds_to_win', default_value='2',
                              description='2 = melhor de três'),
        DeclareLaunchArgument('max_rounds', default_value='5',
                              description='teto duro, para empate em série não '
                                          'travar a fila'),
        DeclareLaunchArgument('round_limit', default_value='90.0',
                              description='teto de um round, em segundos; bem '
                                          'maior que o turno do duelo porque '
                                          'são dois disputando a mesma bola'),
        DeclareLaunchArgument('countdown', default_value='3.0'),
        DeclareLaunchArgument('goal_pause', default_value='4.0',
                              description='comemoração + tempo de recolocar os '
                                          'robôs no centro'),
        DeclareLaunchArgument('round_hold', default_value='5.0',
                              description='quanto o anúncio do round fica no ar'),
        DeclareLaunchArgument('result_hold', default_value='15.0'),

        # ── O campo ──────────────────────────────────────────────────────
        # Sem visão ninguém mede nada com isto, mas o árbitro usa a geometria
        # para desenhar a TV e para o caminho de gol por posição da bola, que
        # continua existindo se alguém rodar este modo com o simulador.
        DeclareLaunchArgument('field_length', default_value='1.50'),
        DeclareLaunchArgument('goal_width', default_value='0.40'),

        # ── Os volantes e a mecânica ─────────────────────────────────────
        DeclareLaunchArgument('deadzone', default_value='0.10'),
        DeclareLaunchArgument('max_linear_velocity', default_value='0.6'),
        DeclareLaunchArgument('max_angular_velocity', default_value='5.0'),
        DeclareLaunchArgument('axle_length', default_value='0.0625'),
        DeclareLaunchArgument('wheel_speed_max', default_value='0.75'),

        # ── O rádio ──────────────────────────────────────────────────────
        # Mesmo nome e mesmo padrão do game.py: quem já decorou o comando de lá
        # não precisa decorar outro aqui. O rádio sobe SEM condição — este modo
        # só existe com robô de verdade em campo.
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        # 30 Hz como nos launches de jogo, não os 60 do teleop: são dois robôs
        # dividindo a mesma ponte, e é o lado do rádio que já deu problema.
        DeclareLaunchArgument('tx_rate_hz', default_value='30.0'),

        LogInfo(msg=['[x1] TV: http://localhost:', LaunchConfiguration('port'),
                     '/  |  Operador: http://localhost:',
                     LaunchConfiguration('port'), '/operador']),
        LogInfo(msg='[x1] duas pessoas, dois robôs (joy_0 -> robô 0, '
                    'joy_1 -> robô 1). Sem câmera e sem IA: quem marca o gol é '
                    'o operador, pelo painel.'),

        Node(
            package='game_master',
            executable='master_node',
            name='game_master',
            output='screen',
            parameters=[{
                'mode': 'x1',
                'port': LaunchConfiguration('port'),
                'rounds_to_win': LaunchConfiguration('rounds_to_win'),
                'max_rounds': LaunchConfiguration('max_rounds'),
                'round_limit': LaunchConfiguration('round_limit'),
                'countdown': LaunchConfiguration('countdown'),
                'goal_pause': LaunchConfiguration('goal_pause'),
                'round_hold': LaunchConfiguration('round_hold'),
                'result_hold': LaunchConfiguration('result_hold'),
                'field_length': field_length,
                'goal_width': goal_width,
            }],
        ),

        *controles,

        Node(
            package='controller_interpreter',
            executable='joy_aggregator',
            name='joy_aggregator',
            output='screen',
            parameters=[{'num_robots': NUM_ROBOTS}],
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
            parameters=[{
                'device_name': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'tx_rate_hz': LaunchConfiguration('tx_rate_hz'),
                'verbose': verbose,
            }],
        ),
    ])
