"""Simulador do campo VSS com GUI no navegador.

Substitui o hardware exatamente na fronteira dele:

    consome  /motorVelocities   (o que iria para o rádio)
    produz   /game_data         (o que a câmera vai publicar)
    produz   /joy_0             (teclado da GUI, no lugar do controle)

Por isso a IA e o game_master escritos contra o simulador funcionam sem
alteração quando os robôs ficarem prontos: basta trocar quem publica /game_data
e quem consome /motorVelocities.

    ros2 run simulator sim_node

Depois abra http://localhost:8080
"""

import asyncio
import json
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, String

from shared_interfaces.msg import AiDebug, GameData, MotorVelocitiesList, RobotState

from aiohttp import web, WSMsgType

from ament_index_python.packages import get_package_share_directory
import os

from . import physics


class SimNode(Node):

    def __init__(self):
        super().__init__('simulator')

        self.declare_parameter('field_length', 1.50)
        self.declare_parameter('field_width', 1.30)
        self.declare_parameter('goal_width', 0.40)
        self.declare_parameter('wheel_speed_max', 0.75)
        self.declare_parameter('axle_length', 0.0625)
        self.declare_parameter('rate_hz', 60.0)
        self.declare_parameter('port', 8080)

        # Convenção do jogo: robô 0 é a IA (defende a esquerda), robô 1 é o
        # visitante (defende a direita). O teclado da GUI publica no /joy_N do
        # jogador, exatamente como faria o controle físico dele.
        self.declare_parameter('player_id', 1)

        # Ruído gaussiano somado às posições publicadas, em metros. A visão real
        # treme; se a IA for afinada contra posições perfeitas ela fica nervosa
        # quando encontrar a câmera. Deixe em 0 para depurar a IA, e ligue para
        # validar que ela aguenta.
        self.declare_parameter('vision_noise', 0.0)

        # Atraso na publicação de /game_data, em segundos. Mesma ideia: a câmera
        # e o processamento custam tempo.
        self.declare_parameter('vision_delay', 0.0)

        # Com o game_master no ar, o árbitro é ele: o simulador não conta gol
        # nem recoloca por conta própria, senão os dois contam o mesmo lance e o
        # placar dobra. Sozinho (para brincar ou depurar), o simulador se vira.
        self.declare_parameter('auto_referee', True)

        field_spec = physics.FieldSpec(
            length=self.get_parameter('field_length').value,
            width=self.get_parameter('field_width').value,
            goal_width=self.get_parameter('goal_width').value,
        )

        robot_spec = physics.RobotSpec(
            wheel_speed_max=self.get_parameter('wheel_speed_max').value,
            wheel_base=self.get_parameter('axle_length').value,
        )

        self.world = physics.make_default_world(field_spec, robot_spec)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.port = int(self.get_parameter('port').value)
        self.noise = float(self.get_parameter('vision_noise').value)
        self.delay = float(self.get_parameter('vision_delay').value)

        self._lock = threading.Lock()
        self._delay_buffer = []
        self._score = {'left': 0, 'right': 0}
        self._paused = False

        # Estado do teclado que veio do navegador, traduzido para eixos de Joy.
        self._keys = set()

        self.create_subscription(
            MotorVelocitiesList, '/motorVelocities', self._on_motor_velocities, 10)

        # Espelha o raciocínio da IA na GUI. É a mesma informação que vai para a
        # TV na feira — vale desenvolver contra ela desde já.
        self._ai_debug = None
        self.create_subscription(AiDebug, '/ai/debug', self._on_ai_debug, 10)

        self._auto_referee = bool(self.get_parameter('auto_referee').value)
        self.create_subscription(Empty, '/sim/reset', self._on_reset_request, 10)

        self._player_id = int(self.get_parameter('player_id').value)
        joy_topic = f'/joy_{self._player_id}'

        self._game_data_pub = self.create_publisher(GameData, '/game_data', 10)
        self._joy_pub = self.create_publisher(Joy, joy_topic, 10)
        self._difficulty_pub = self.create_publisher(String, '/ai/difficulty', 10)

        self.create_timer(1.0 / self.rate, self._tick)

        self._ws_clients = set()
        self._loop = None

        self._start_web_server()

        self.get_logger().info(
            f'Simulador rodando. Abra http://localhost:{self.port}')
        self.get_logger().info(
            f'Teclado da GUI -> {joy_topic} (robô {self._player_id}, o jogador)')

    # ── ROS ──────────────────────────────────────────────────────────────

    def _on_motor_velocities(self, msg):
        with self._lock:
            for velocity in msg.velocities:
                physics.set_wheel_command(
                    self.world, velocity.id, velocity.left, velocity.right)

    def _on_reset_request(self, _msg):
        with self._lock:
            physics.reset_positions(self.world)

    def _on_ai_debug(self, msg):
        self._ai_debug = {
            'state': msg.state,
            'target_x': msg.target_x,
            'target_y': msg.target_y,
            'ball_x': msg.ball_x,
            'ball_y': msg.ball_y,
            'difficulty': msg.difficulty,
            'reaction_delay': msg.reaction_delay,
            'speed_frac': msg.speed_frac,
        }

    def _tick(self):
        dt = 1.0 / self.rate

        with self._lock:
            if not self._paused:
                physics.step(self.world, dt)

                # Só apita se estiver sozinho. Com o game_master no ar, ele é
                # quem conta o gol e pede a recolocação por /sim/reset.
                if self.world.goal_event and self._auto_referee:
                    self._score[self.world.goal_event] += 1
                    self.get_logger().info(
                        f'GOL {self.world.goal_event} | '
                        f"{self._score['left']} x {self._score['right']}")
                    physics.reset_positions(self.world)

            snapshot = self._snapshot()

        self._publish_joy()
        self._publish_game_data(snapshot)
        self._broadcast(snapshot)

    def _snapshot(self):
        """Fotografia do mundo, já em dicionário, para publicar e desenhar."""
        w = self.world
        return {
            'ball': {'x': w.ball.x, 'y': w.ball.y, 'vx': w.ball.vx, 'vy': w.ball.vy},
            'robots': [
                {
                    'id': r.id, 'x': r.x, 'y': r.y, 'theta': r.theta,
                    'vx': r.vx, 'vy': r.vy, 'vtheta': r.vtheta,
                    'left': r.cmd_left, 'right': r.cmd_right,
                }
                for r in sorted(w.robots.values(), key=lambda r: r.id)
            ],
            'field': {
                'length': w.field.length,
                'width': w.field.width,
                'goal_width': w.field.goal_width,
                'robot_radius': w.robot_spec.radius,
                'ball_radius': w.ball_spec.radius,
            },
            'score': dict(self._score),
            'paused': self._paused,
            'ai': self._ai_debug,
        }

    def _publish_game_data(self, snapshot):
        # Aplica o atraso configurado guardando snapshots numa fila.
        if self.delay > 0:
            self._delay_buffer.append((self.get_clock().now(), snapshot))
            cutoff = self.get_clock().now() - rclpy.duration.Duration(seconds=self.delay)

            ready = None
            while self._delay_buffer and self._delay_buffer[0][0] <= cutoff:
                ready = self._delay_buffer.pop(0)[1]

            if ready is None:
                return
            snapshot = ready

        msg = GameData()
        msg.stamp = self.get_clock().now().to_msg()
        msg.ball_detected = True

        msg.ball.x = self._noisy(snapshot['ball']['x'])
        msg.ball.y = self._noisy(snapshot['ball']['y'])
        msg.ball.vx = snapshot['ball']['vx']
        msg.ball.vy = snapshot['ball']['vy']

        for robot in snapshot['robots']:
            state = RobotState()
            state.id = robot['id']
            state.x = self._noisy(robot['x'])
            state.y = self._noisy(robot['y'])
            state.orientation = robot['theta']
            state.vx = robot['vx']
            state.vy = robot['vy']
            state.vtheta = robot['vtheta']
            msg.robots.append(state)

        self._game_data_pub.publish(msg)

    def _noisy(self, value):
        if self.noise <= 0:
            return value
        import random
        return value + random.gauss(0.0, self.noise)

    def _publish_joy(self):
        """Traduz as teclas do navegador em Joy, no formato que o pipeline espera."""
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0] * 6
        msg.buttons = [0] * 3

        # Convenção "signed": solto = +1.0, apertado = -1.0.
        msg.axes[4] = 1.0
        msg.axes[5] = 1.0

        keys = set(self._keys)

        if 'w' in keys:
            msg.axes[5] = -1.0
        if 's' in keys:
            msg.axes[4] = -1.0
        if 'd' in keys:
            msg.axes[0] = 1.0
        if 'a' in keys:
            msg.axes[0] = -1.0
        if 'e' in keys:
            msg.buttons[0] = 1
        if 'q' in keys:
            msg.buttons[2] = 1
        if 'b' in keys:
            msg.buttons[1] = 1

        self._joy_pub.publish(msg)

    # ── Comandos vindos da GUI ───────────────────────────────────────────

    def _handle_command(self, data):
        kind = data.get('type')

        if kind == 'keys':
            self._keys = set(data.get('keys', []))

        elif kind == 'reset':
            with self._lock:
                physics.reset_positions(self.world)

        elif kind == 'reset_score':
            self._score = {'left': 0, 'right': 0}

        elif kind == 'pause':
            self._paused = not self._paused

        elif kind == 'place_ball':
            with self._lock:
                self.world.ball.x = float(data['x'])
                self.world.ball.y = float(data['y'])
                self.world.ball.vx = 0.0
                self.world.ball.vy = 0.0

        elif kind == 'place_robot':
            with self._lock:
                robot = self.world.robots.get(int(data['id']))
                if robot:
                    robot.x = float(data['x'])
                    robot.y = float(data['y'])
                    robot.v = 0.0
                    robot.omega = 0.0

        elif kind == 'difficulty':
            msg = String()
            msg.data = str(data.get('value', 'MEDIO'))
            self._difficulty_pub.publish(msg)

        elif kind == 'kick_ball':
            with self._lock:
                self.world.ball.vx = float(data.get('vx', 0.0))
                self.world.ball.vy = float(data.get('vy', 0.0))

    # ── Servidor web ─────────────────────────────────────────────────────

    def _start_web_server(self):
        """Sobe o aiohttp num thread próprio, com seu próprio event loop.

        rclpy roda em loop síncrono e o aiohttp precisa de asyncio; separar os
        dois em threads é mais simples e mais robusto do que tentar casar os
        dois schedulers.
        """
        thread = threading.Thread(target=self._web_thread_guard,
                                  args=(self._run_web_server,), daemon=True)
        thread.start()

    def _web_thread_guard(self, run):
        """Roda o servidor web e NÃO deixa o thread morrer calado.

        Este thread é daemon. Sem esta guarda, qualquer erro ao montar as rotas
        — uma rota malformada, um caminho que não existe — mata só o thread: o
        nó segue vivo, publicando tudo normalmente, e a porta simplesmente
        nunca abre. O navegador responde "connection refused", que não aponta
        para lugar nenhum, e o `--check` mostra o nó rodando e a lista de
        portas vazia.

        Já aconteceu exatamente assim, com `/fonts/{{name}}` escrito com
        chave dupla. O traceback existia e ficava só dentro do thread.
        """
        import traceback
        try:
            run()
        except Exception:
            self.get_logger().fatal(
                'servidor web do simulador caiu ao subir e a interface não vai abrir:\n'
                + traceback.format_exc())
            os._exit(1)

    def _run_web_server(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        app = web.Application()
        app.router.add_get('/', self._serve_index)
        app.router.add_get('/vss.css', self._serve_css)
        # Fontes auto-hospedadas. A feira não tem rede garantida e a
        # identidade do telão depende delas — nada de CDN.
        app.router.add_get('/fonts/{name}', self._serve_font)
        app.router.add_get('/ws', self._websocket_handler)

        runner = web.AppRunner(app)
        self._loop.run_until_complete(runner.setup())

        site = web.TCPSite(runner, '0.0.0.0', self.port)

        try:
            self._loop.run_until_complete(site.start())
        except OSError as exc:
            # Quase sempre é uma instância anterior que não morreu. Este thread
            # morreria em silêncio e o nó seguiria rodando sem GUI, publicando
            # /game_data e disputando os tópicos com a instância antiga — que é
            # exatamente o tipo de falha que custa meia hora para diagnosticar.
            self.get_logger().fatal(
                f'Não consegui abrir a porta {self.port}: {exc}\n'
                f'Provavelmente já há um simulador rodando. Verifique com:\n'
                f'  ss -tlnp | grep {self.port}\n'
                f'  ./tools/sim.sh status')
            os._exit(1)

        self._loop.run_forever()

    async def _serve_index(self, request):
        share = get_package_share_directory('simulator')
        path = os.path.join(share, 'web', 'index.html')
        return web.FileResponse(path)

    async def _serve_css(self, request):
        # Identidade visual compartilhada. O arquivo de verdade é web/vss.css
        # na raiz do repositório; o que está no pacote é um symlink para lá,
        # então TV, operador, simulador e calibração mudam juntos.
        share = get_package_share_directory('simulator')
        return web.FileResponse(os.path.join(share, 'web', 'vss.css'))

    #: Nomes de fonte servidos em /fonts. Fechado de propósito — ver _serve_font.
    async def _serve_font(self, request):
        """Fonte auto-hospedada, uma por vez.

        Não usa `add_static`: com `colcon build --symlink-install` cada .woff2
        instalado é um symlink para o repositório, e o `add_static` do aiohttp
        recusa servir através de symlink (`follow_symlinks=False`). O sintoma é
        cruel — 404 em toda fonte, a tela cai calada para a fonte do sistema e
        nada aparece no log do nó. O `FileResponse` atravessa o symlink, que é
        o mesmo motivo de o /vss.css nunca ter dado problema.

        O nome vem da URL, então é conferido antes de virar caminho: só arquivo
        que esteja de fato dentro do diretório de fontes é servido.
        """
        base = os.path.join(
            get_package_share_directory('simulator'), 'web', 'fonts')
        target = os.path.normpath(os.path.join(base, request.match_info['name']))

        if os.path.dirname(target) != os.path.normpath(base) or not os.path.isfile(target):
            raise web.HTTPNotFound()

        # O `mimetypes` do Python não conhece .woff2 e devolveria
        # application/octet-stream. Navegador aceita assim, mas declarar o tipo
        # certo é de graça e tira o palpite do caminho.
        return web.FileResponse(
            target, headers={'Content-Type': 'font/woff2',
                             'Cache-Control': 'public, max-age=86400'})

    async def _websocket_handler(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        self._ws_clients.add(ws)

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        self._handle_command(json.loads(msg.data))
                    except (ValueError, KeyError) as exc:
                        self.get_logger().warn(f'Comando inválido da GUI: {exc}')
        finally:
            self._ws_clients.discard(ws)

        return ws

    def _broadcast(self, snapshot):
        if not self._ws_clients or self._loop is None:
            return

        payload = json.dumps(snapshot)

        async def send_all():
            dead = []
            for ws in list(self._ws_clients):
                try:
                    await ws.send_str(payload)
                except (ConnectionError, RuntimeError):
                    dead.append(ws)
            for ws in dead:
                self._ws_clients.discard(ws)

        asyncio.run_coroutine_threadsafe(send_all(), self._loop)


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()

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
