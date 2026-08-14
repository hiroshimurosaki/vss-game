"""O adversário. Publica um Joy sintético, como se fosse alguém no controle.

    consome  /game_data      (visão, ou o simulador)
    produz   /joy_<robot_id> (o mesmo tópico que um controle físico usaria)
    produz   /ai/debug       (o que ela está pensando, para a TV mostrar)

Publicar Joy em vez de ir direto ao motor é deliberado: a IA passa pelo
joy_aggregator, direction, special_controls e cinematica exatamente como o
jogador humano. Ela sofre as mesmas saturações e os mesmos limites. Se alguém
mexer na cinemática, mexe para os dois.

    ros2 run ai_player ai_node --ros-args -p difficulty:=FACIL
"""

import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from shared_interfaces.msg import AiDebug, GameData

from . import brain


class _Point:
    """Adaptador mínimo para o brain não depender das mensagens do ROS."""

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta


class AiNode(Node):

    def __init__(self):
        super().__init__('ai_player')

        self.declare_parameter('robot_id', 0)
        self.declare_parameter('difficulty', 'MEDIO')

        # Qual conjunto de presets: 'jogo' (dois robôs, com adversário em campo)
        # ou 'duelo' (turno solo). Os números são diferentes porque o
        # comportamento defensivo e o erro de mira se comportam de outro jeito
        # sem adversário — ver o comentário de PRESETS_DUELO em brain.py.
        self.declare_parameter('preset_set', 'jogo')
        self.declare_parameter('rate_hz', 30.0)

        self.declare_parameter('field_length', 1.50)
        self.declare_parameter('field_width', 1.30)
        self.declare_parameter('goal_width', 0.40)

        # Modo duelo: para onde a IA leva o robô entre um turno e outro, quando
        # /ai/home manda. -99.0 = a marca de saída do próprio robô, calculada do
        # campo — a mesma que o simulador usa no kickoff.
        self.declare_parameter('home_x', -99.0)
        self.declare_parameter('home_y', 0.0)
        # Recolocar não é jogar: vai mais devagar que o máximo porque alguém
        # está com a mão no campo repondo a bola enquanto ele atravessa.
        self.declare_parameter('home_speed_frac', 0.7)

        # Precisam bater com os do DirectionNode: são o que traduz a decisão
        # em posição de gatilho.
        self.declare_parameter('max_linear_velocity', 0.6)
        self.declare_parameter('max_angular_velocity', 5.0)

        # Sobrescritas individuais do preset. -1.0 significa "usa o do preset".
        # Servem para afinar ao vivo na feira sem editar código:
        #   ros2 param set /ai_player speed_frac 0.5
        self.declare_parameter('speed_frac', -1.0)
        self.declare_parameter('reaction_delay', -1.0)
        self.declare_parameter('replan_period', -1.0)
        self.declare_parameter('home_x_max', -99.0)
        self.declare_parameter('aim_noise', -1.0)

        self._robot_id = int(self.get_parameter('robot_id').value)

        self._geo = brain.Geometry(
            half_length=self.get_parameter('field_length').value / 2.0,
            half_width=self.get_parameter('field_width').value / 2.0,
            goal_half=self.get_parameter('goal_width').value / 2.0,
            max_linear=self.get_parameter('max_linear_velocity').value,
            max_angular=self.get_parameter('max_angular_velocity').value,
        )

        conjunto = str(self.get_parameter('preset_set').value).lower()

        if conjunto not in brain.CONJUNTOS:
            self.get_logger().warn(
                f"preset_set '{conjunto}' desconhecido. Usando 'jogo'. "
                f"Opções: {', '.join(brain.CONJUNTOS)}")
            conjunto = 'jogo'

        self._preset_set = conjunto
        self._presets = brain.CONJUNTOS[conjunto]

        self._diff = self._build_difficulty()

        home_x = float(self.get_parameter('home_x').value)
        if home_x <= -99.0:
            home_x = -self._geo.half_length * 0.55

        self._home = (home_x, float(self.get_parameter('home_y').value))
        self._home_speed = float(self.get_parameter('home_speed_frac').value)
        self._home_mode = False

        self._enabled = True
        self._perception_buffer = []
        self._last_plan_time = None
        self._cached_target = None
        self._last_state = None

        joy_topic = f'/joy_{self._robot_id}'

        self.create_subscription(GameData, '/game_data', self._on_game_data, 10)
        self.create_subscription(Bool, '/ai/enabled', self._on_enabled, 10)
        self.create_subscription(Bool, '/ai/home', self._on_home, 10)
        self.create_subscription(String, '/ai/difficulty', self._on_difficulty, 10)

        self._joy_pub = self.create_publisher(Joy, joy_topic, 10)
        self._debug_pub = self.create_publisher(AiDebug, '/ai/debug', 10)

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'IA no robô {self._robot_id} -> {joy_topic} | '
            f'presets "{self._preset_set}" | dificuldade {self._diff.name}')
        self._log_difficulty()

    # ── Configuração ─────────────────────────────────────────────────────

    def _build_difficulty(self):
        return self._preset(str(self.get_parameter('difficulty').value))

    def _preset(self, name, avisa=True):
        """Monta a dificuldade pelo nome, já com as sobrescritas aplicadas."""
        name = name.upper()
        presets = self._presets

        if name not in presets:
            if avisa:
                self.get_logger().warn(
                    f"Dificuldade '{name}' desconhecida. Usando MEDIO. "
                    f"Opções: {', '.join(presets)}")
            name = 'MEDIO'

        # Cópia, para as sobrescritas não contaminarem o preset global.
        diff = brain.Difficulty(**vars(presets[name]))

        overrides = [
            ('speed_frac', 'speed_frac', -1.0),
            ('reaction_delay', 'reaction_delay', -1.0),
            ('replan_period', 'replan_period', -1.0),
            ('home_x_max', 'home_x_max', -99.0),
            ('aim_noise', 'aim_noise', -1.0),
        ]

        for param, attr, sentinel in overrides:
            value = float(self.get_parameter(param).value)
            if value != sentinel:
                setattr(diff, attr, value)

        return diff

    def _log_difficulty(self):
        d = self._diff
        self.get_logger().info(
            f'  velocidade  {d.speed_frac:.0%} do máximo\n'
            f'  reação      {d.reaction_delay * 1000:.0f} ms de atraso\n'
            f'  replaneja   a cada {d.replan_period * 1000:.0f} ms\n'
            f'  avança até  x = {d.home_x_max:+.2f} m\n'
            f'  erro de mira {d.aim_noise * 100:.0f} cm')

    def _on_difficulty(self, msg):
        """Permite trocar a dificuldade em runtime, sem reiniciar nada.

        Passa pelo mesmo `_preset` da inicialização, e por um motivo concreto:
        antes esta função reconstruía do preset cru e **jogava as sobrescritas
        fora**. Quem tivesse afinado a IA por `ros2 param set` perdia o ajuste
        no instante em que o operador tocasse nos botões de dificuldade da tela,
        sem nada no log dizendo o que aconteceu.
        """
        name = msg.data.upper()

        if name not in self._presets:
            self.get_logger().warn(f"Dificuldade '{name}' desconhecida.")
            return

        self._diff = self._preset(name)
        self._cached_target = None

        self.get_logger().info(f'Dificuldade trocada para {name}')
        self._log_difficulty()

    def _on_enabled(self, msg):
        if msg.data != self._enabled:
            self.get_logger().info(
                'IA ' + ('liberada' if msg.data else 'congelada'))
        self._enabled = msg.data

        if not msg.data:
            self._cached_target = None

    def _on_home(self, msg):
        """Modo duelo: leva o robô à marca em vez de jogar.

        Não substitui o /ai/enabled — some com ele. Recolocar só acontece com a
        IA liberada, então o árbitro precisa mandar os dois.
        """
        if msg.data != self._home_mode:
            self.get_logger().info(
                'IA ' + ('voltando à marca' if msg.data else 'jogando'))
        self._home_mode = msg.data
        self._cached_target = None

    # ── Percepção ────────────────────────────────────────────────────────

    def _on_game_data(self, msg):
        """Guarda o snapshot com carimbo, para poder entregá-lo atrasado."""
        self._perception_buffer.append((self.get_clock().now(), msg))

        # Não deixa a fila crescer sem limite se o atraso for zero.
        if len(self._perception_buffer) > 240:
            self._perception_buffer.pop(0)

    def _current_perception(self):
        """O que a IA enxerga agora — que é o passado, se houver atraso."""
        if not self._perception_buffer:
            return None

        if self._diff.reaction_delay <= 0:
            return self._perception_buffer[-1][1]

        cutoff = (self.get_clock().now()
                  - rclpy.duration.Duration(seconds=self._diff.reaction_delay))

        chosen = None
        while (self._perception_buffer
               and self._perception_buffer[0][0] <= cutoff):
            chosen = self._perception_buffer.pop(0)[1]

        # Nada velho o bastante ainda: a IA literalmente não viu nada.
        return chosen

    # ── Loop ─────────────────────────────────────────────────────────────

    def _tick(self):
        if not self._enabled:
            self._publish_joy(0.0, 0.0)
            self._publish_debug(brain.Decision(state=brain.PARADO), None)
            return

        perception = self._current_perception()

        if perception is None:
            self._publish_joy(0.0, 0.0)
            return

        me = None
        for robot in perception.robots:
            if robot.id == self._robot_id:
                me = _Point(robot.x, robot.y, robot.orientation)
                break

        if me is None:
            # Sem se enxergar, ela para. Um robô que age com informação vencida
            # bate na parede.
            self._publish_joy(0.0, 0.0)
            self._publish_debug(brain.Decision(state=brain.PARADO), perception)
            return

        # Voltar à marca não depende da bola: é justamente o momento em que
        # alguém está com a mão no campo repondo ela.
        if self._home_mode:
            self._drive_home(me, perception)
            return

        if not perception.ball_detected:
            self._publish_joy(0.0, 0.0)
            self._publish_debug(brain.Decision(state=brain.PARADO), perception)
            return

        ball = _Point(perception.ball.x, perception.ball.y)

        # Replanejamento com período: entre uma decisão e outra ela persegue o
        # alvo antigo. É o que faz a IA parecer humana em vez de teleguiada.
        now = self.get_clock().now()
        should_replan = (
            self._last_plan_time is None
            or (now - self._last_plan_time).nanoseconds / 1e9 >= self._diff.replan_period
        )

        if should_replan:
            self._last_plan_time = now
            self._cached_target = None

        noise = (0.0, 0.0)
        if self._cached_target is None and self._diff.aim_noise > 0:
            noise = (random.gauss(0.0, self._diff.aim_noise),
                     random.gauss(0.0, self._diff.aim_noise))

        decision = brain.decide(
            ball, me, self._geo, self._diff,
            cached_target=self._cached_target, noise=noise,
            prev_state=self._last_state)

        self._cached_target = (decision.target_x, decision.target_y, decision.state)
        self._last_state = decision.state

        self._publish_joy(decision.linear, decision.angular)
        self._publish_debug(decision, perception)

    def _drive_home(self, me, perception):
        """Leva o robô de volta à marca, entre os turnos do duelo.

        Usa o mesmo `go_to_point` do jogo, com freio ligado — aqui o objetivo é
        parar em cima do ponto, não atravessá-lo. A dificuldade NÃO entra: um
        preset FACIL faria o robô rastejar de volta e a fila esperaria por uma
        escolha que só deveria valer enquanto ele joga.
        """
        home_x, home_y = self._home

        linear, angular = brain.go_to_point(
            me, home_x, home_y, self._geo, self._diff, brake=True)

        linear *= self._home_speed
        angular *= self._home_speed

        self._publish_joy(linear, angular)
        self._publish_debug(
            brain.Decision(linear=linear, angular=angular,
                           state=brain.VOLTANDO,
                           target_x=home_x, target_y=home_y),
            perception)

    def _publish_joy(self, linear, angular):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = brain.to_joy_axes(linear, angular, self._geo)
        msg.buttons = [0, 0, 0]
        self._joy_pub.publish(msg)

    def _publish_debug(self, decision, perception):
        msg = AiDebug()
        msg.state = decision.state
        msg.target_x = decision.target_x
        msg.target_y = decision.target_y
        msg.linear_vel = decision.linear
        msg.angular_vel = decision.angular
        msg.difficulty = self._diff.name
        msg.reaction_delay = self._diff.reaction_delay
        msg.speed_frac = self._diff.speed_frac

        if perception is not None:
            msg.ball_x = perception.ball.x
            msg.ball_y = perception.ball.y

        self._debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AiNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
