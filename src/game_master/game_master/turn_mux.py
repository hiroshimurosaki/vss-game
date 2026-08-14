"""O revezamento do volante: duas fontes de Joy, um robô, uma saída.

    consome  /duelo/joy_humano   (gamepad ou teclado do visitante)
    consome  /duelo/joy_ia       (a IA)
    consome  /game/joy_source    (o árbitro dizendo de quem é a vez)
    produz   /joy_<robot_id>     (o mesmo tópico que um controle físico usaria)

## Por que este nó existe

No modo duelo o visitante e a IA dirigem o MESMO robô, em turnos. A tentação é
deixar os dois publicando direto no `/joy_0` e calar o que não está na vez —
e é exatamente o erro que este projeto já cometeu duas vezes. Dois produtores no
mesmo `/joy_N` fazem o robô gaguejar **sem nada no log explicando**, porque o
que chega no `joy_aggregator` é a intercalação dos dois: um comando de verdade,
um zero, um comando, um zero. "Calar" também não resolve, porque tanto o
`game_controller_node` (autorepeat) quanto a IA (que publica zeros quando
congelada) continuam falando quando não têm nada a dizer.

A saída é não ter dois produtores. As duas fontes vão para tópicos privados e
**este nó é o único publicador** do tópico que o robô ouve. Quem não está na vez
não é silenciado: é ignorado.

## O que ele garante

- **Taxa fixa de saída.** Publica sempre, a `rate_hz`, independente do que as
  fontes fizerem. O robô nunca fica sem comando por causa de uma fonte lenta.
- **Watchdog próprio.** Se a fonte da vez ficar mais de `input_timeout` sem
  falar, a saída vai a neutro. Vale para o gamepad que desconectou no meio do
  turno tanto quanto para a IA que morreu.
- **Corte limpo na troca.** Ao mudar de motorista, o último comando do anterior
  é descartado na hora. Sem isso o robô herdaria o acelerador de quem acabou de
  sair do volante — a IA termina o turno em velocidade máxima, e o visitante
  receberia o robô já correndo.

Neutro é o array zerado: é a convenção "sdl" dos gatilhos (solto = 0,0), então o
próprio zero do array já é o repouso e não há o que preencher.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


JOGADOR = 'JOGADOR'
IA = 'IA'

# 6 eixos porque é o que o DirectionNode lê: axes[0] volante, axes[4] L2 (ré) e
# axes[5] R2 (frente). Zerado é o repouso na convenção sdl.
NEUTRO = [0.0] * 6


class TurnMux(Node):

    def __init__(self):
        super().__init__('turn_mux')

        self.declare_parameter('robot_id', 0)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('input_timeout', 0.3)
        self.declare_parameter('verbose', False)

        self._robot_id = int(self.get_parameter('robot_id').value)
        self._timeout = float(self.get_parameter('input_timeout').value)
        self._verbose = bool(self.get_parameter('verbose').value)

        self._source = ''
        self._held = None
        self._held_at = None

        self.create_subscription(Joy, '/duelo/joy_humano',
                                 self._on_humano, 10)
        self.create_subscription(Joy, '/duelo/joy_ia', self._on_ia, 10)
        self.create_subscription(String, '/game/joy_source',
                                 self._on_source, 10)

        topic = f'/joy_{self._robot_id}'
        self._pub = self.create_publisher(Joy, topic, 10)

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'Revezamento do volante -> {topic} a {rate:.0f} Hz.\n'
            f'  humano: /duelo/joy_humano | IA: /duelo/joy_ia\n'
            f'  de quem é a vez: /game/joy_source')

    # ── Entradas ─────────────────────────────────────────────────────────

    def _on_source(self, msg):
        novo = msg.data.strip().upper()

        if novo == self._source:
            return

        self.get_logger().info(
            f'Volante: {self._source or "ninguém"} -> {novo or "ninguém"}')

        self._source = novo

        # Descarta o comando do motorista anterior. Ver docstring: sem isto o
        # robô entra no turno seguinte com o acelerador do turno anterior.
        self._held = None
        self._held_at = None

    def _on_humano(self, msg):
        if self._source == JOGADOR:
            self._hold(msg)

    def _on_ia(self, msg):
        if self._source == IA:
            self._hold(msg)

    def _hold(self, msg):
        self._held = msg
        self._held_at = self.get_clock().now()

    # ── Saída ────────────────────────────────────────────────────────────

    def _tick(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self._fresh():
            msg.axes = list(self._held.axes)
            msg.buttons = list(self._held.buttons)
        else:
            msg.axes = list(NEUTRO)
            msg.buttons = [0, 0, 0]

        self._pub.publish(msg)

    def _fresh(self):
        if self._held is None or self._held_at is None:
            return False

        if self._timeout <= 0:
            return True

        idade = (self.get_clock().now() - self._held_at).nanoseconds / 1e9

        if idade > self._timeout:
            if self._verbose:
                self.get_logger().warn(
                    f'{self._source or "ninguém"} calado há {idade:.2f} s. '
                    f'Motor em neutro.')
            return False

        return True


def main(args=None):
    rclpy.init(args=args)
    node = TurnMux()

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
