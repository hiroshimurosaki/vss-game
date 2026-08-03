"""O árbitro. Conduz a partida, cronometra, guarda o ranking e serve as telas.

    consome  /game_data       (posição da bola -> detecção de gol)
    produz   /game/status     (tudo que as telas desenham)
    produz   /ai/enabled      (congela a IA entre partidas e na comemoração)
    produz   /sim/reset       (pede recolocação; o simulador ouve, o campo real ignora)

Serve duas páginas no mesmo servidor:

    http://localhost:8090/           a TV, para o público
    http://localhost:8090/operador   o painel de quem está tocando o estande

O árbitro é este nó, não o simulador: assim a regra do gol é literalmente o
mesmo código no simulador e no campo real, porque os dois falam /game_data.
"""

import asyncio
import json
import os
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from shared_interfaces.msg import GameData, GameStatus, HighScore, HighScoreList

from aiohttp import web, WSMsgType
from ament_index_python.packages import get_package_share_directory

from . import rules


class GameMaster(Node):

    def __init__(self):
        super().__init__('game_master')

        self.declare_parameter('port', 8090)
        self.declare_parameter('target_score', 2)
        self.declare_parameter('time_limit', 180.0)
        self.declare_parameter('countdown', 3.0)
        self.declare_parameter('goal_pause', 4.0)
        self.declare_parameter('result_hold', 12.0)

        self.declare_parameter('field_length', 1.50)
        self.declare_parameter('goal_width', 0.40)

        # Margem para dentro do gol antes de considerar bola dentro. Com a visão
        # real, um pouco de folga evita gol fantasma quando a bola raspa a linha.
        self.declare_parameter('goal_margin', 0.0)

        self.declare_parameter('difficulty', 'MEDIO')

        default_scores = os.path.join(os.path.expanduser('~'), '.vss-game',
                                      'highscores.json')
        self.declare_parameter('scores_file', default_scores)

        config = rules.Config(
            target_score=int(self.get_parameter('target_score').value),
            time_limit=float(self.get_parameter('time_limit').value),
            countdown=float(self.get_parameter('countdown').value),
            goal_pause=float(self.get_parameter('goal_pause').value),
            result_hold=float(self.get_parameter('result_hold').value),
        )

        self.engine = rules.Engine(config=config)

        self._half_length = float(self.get_parameter('field_length').value) / 2.0
        self._goal_half = float(self.get_parameter('goal_width').value) / 2.0
        self._goal_margin = float(self.get_parameter('goal_margin').value)
        self._difficulty = str(self.get_parameter('difficulty').value).upper()

        self._scores_path = str(self.get_parameter('scores_file').value)
        self._scores = self._load_scores()

        self._lock = threading.Lock()
        self._last_state = None
        self._pending_name = ''

        self.create_subscription(GameData, '/game_data', self._on_game_data, 10)
        self.create_subscription(String, '/ai/difficulty',
                                 self._on_difficulty, 10)

        self._status_pub = self.create_publisher(GameStatus, '/game/status', 10)
        self._scores_pub = self.create_publisher(HighScoreList, '/game/highscores', 10)
        self._ai_enabled_pub = self.create_publisher(Bool, '/ai/enabled', 10)
        self._difficulty_pub = self.create_publisher(String, '/ai/difficulty', 10)
        self._sim_reset_pub = self.create_publisher(Empty, '/sim/reset', 10)

        self.create_timer(1.0 / 30.0, self._tick)

        self.port = int(self.get_parameter('port').value)
        self._ws_clients = set()
        self._loop = None
        self._start_web_server()

        self.get_logger().info(
            f'Árbitro no ar.\n'
            f'  TV:       http://localhost:{self.port}/\n'
            f'  Operador: http://localhost:{self.port}/operador\n'
            f'  Ranking:  {self._scores_path} ({len(self._scores)} tempos)')

    # ── Relógio ──────────────────────────────────────────────────────────

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ── ROS ──────────────────────────────────────────────────────────────

    def _on_difficulty(self, msg):
        self._difficulty = msg.data.upper()

    def _on_game_data(self, msg):
        if not msg.ball_detected:
            return

        with self._lock:
            scorer = self.engine.on_ball(
                self._now(), msg.ball.x, msg.ball.y,
                self._half_length, self._goal_half, self._goal_margin)

        if scorer:
            m = self.engine.match
            self.get_logger().info(
                f'GOL do {scorer} | {m.player_score} x {m.ai_score} | '
                f'{rules.format_time(m.elapsed)}')

    def _tick(self):
        now = self._now()

        with self._lock:
            match = self.engine.tick(now)

            # Entrou num estado novo? Alguns exigem uma ação de uma vez só.
            if match.state != self._last_state:
                self._on_state_change(self._last_state, match.state)
                self._last_state = match.state

            status = self._build_status(now, match)

        self._status_pub.publish(status)

        # A IA só joga durante a partida. Fora disso ela fica parada, senão
        # empurra a bola pelo campo enquanto o próximo jogador digita o nome.
        enabled = Bool()
        enabled.data = (match.state == rules.JOGANDO)
        self._ai_enabled_pub.publish(enabled)

        self._broadcast(self._snapshot(status))

    def _on_state_change(self, old, new):
        # Recolocar no início da contagem e depois de cada gol. No simulador
        # isso teletransporta; no campo real é um pedido para o operador — que
        # tem a duração do goal_pause para fazer.
        if new in (rules.CONTAGEM, rules.GOL):
            self._sim_reset_pub.publish(Empty())

        if new == rules.FIM:
            self._commit_result()

    def _commit_result(self):
        """Fecha a partida: grava no ranking se o jogador venceu."""
        m = self.engine.match

        if not m.player_won:
            self.get_logger().info(
                f'{m.player_name} não venceu ({m.player_score}x{m.ai_score}). '
                f'Fora do ranking.')
            return

        self._scores, position = rules.insert_score(
            self._scores, m.player_name, m.final_time,
            self._difficulty, datetime.now().strftime('%Y-%m-%d %H:%M'))

        m.ranked = position > 0
        m.rank_position = position

        self._save_scores()
        self._publish_scores()

        self.get_logger().info(
            f'{m.player_name} venceu em {rules.format_time(m.final_time)} '
            + (f'-> {position}º lugar!' if position else '-> fora do top 10'))

    def _build_status(self, now, match):
        status = GameStatus()
        status.state = match.state
        status.player_name = match.player_name
        status.player_score = match.player_score
        status.ai_score = match.ai_score
        status.target_score = self.engine.config.target_score
        status.elapsed = match.elapsed
        status.time_limit = self.engine.config.time_limit
        status.state_remaining = self.engine.state_remaining(now)
        status.player_won = match.player_won
        status.ranked = match.ranked
        status.rank_position = match.rank_position
        status.last_scorer = match.last_scorer
        status.difficulty = self._difficulty
        return status

    # ── Ranking em disco ─────────────────────────────────────────────────

    def _load_scores(self):
        try:
            with open(self._scores_path) as handle:
                data = json.load(handle)
                return data if isinstance(data, list) else []
        except FileNotFoundError:
            return []
        except (json.JSONDecodeError, OSError) as exc:
            # Nunca deixar o ranking corrompido derrubar o jogo no meio da
            # feira: começa vazio e segue. O arquivo velho fica para trás.
            self.get_logger().error(
                f'Não consegui ler {self._scores_path}: {exc}. Começando vazio.')
            return []

    def _save_scores(self):
        try:
            os.makedirs(os.path.dirname(self._scores_path), exist_ok=True)

            # Grava em arquivo temporário e move: se acabar a energia no meio,
            # o ranking antigo continua íntegro em vez de virar lixo.
            temp = self._scores_path + '.tmp'
            with open(temp, 'w') as handle:
                json.dump(self._scores, handle, ensure_ascii=False, indent=2)
            os.replace(temp, self._scores_path)

        except OSError as exc:
            self.get_logger().error(f'Falha ao salvar o ranking: {exc}')

    def _publish_scores(self):
        msg = HighScoreList()
        for item in self._scores:
            entry = HighScore()
            entry.name = item['name']
            entry.time = float(item['time'])
            entry.difficulty = item.get('difficulty', '')
            entry.date = item.get('date', '')
            msg.entries.append(entry)
        self._scores_pub.publish(msg)

    # ── Comandos das telas ───────────────────────────────────────────────

    def _handle_command(self, data):
        kind = data.get('type')
        now = self._now()

        with self._lock:
            if kind == 'register':
                self.engine.begin_registration(now)

            elif kind == 'start':
                self.engine.start(now, data.get('name', ''))
                self.get_logger().info(
                    f'Partida iniciada: {self.engine.match.player_name}')

            elif kind == 'abort':
                self.engine.abort(now)

            elif kind == 'pause':
                self.engine.toggle_pause(now)

            elif kind == 'goal':
                scorer = (rules.JOGADOR if data.get('who') == 'player'
                          else rules.IA)
                self.engine.force_goal(now, scorer)
                self.get_logger().info(f'Gol manual do árbitro: {scorer}')

            elif kind == 'difficulty':
                msg = String()
                msg.data = str(data.get('value', 'MEDIO')).upper()
                self._difficulty_pub.publish(msg)
                self._difficulty = msg.data

            elif kind == 'clear_scores':
                self._scores = []
                self._save_scores()
                self._publish_scores()
                self.get_logger().warn('Ranking apagado pelo operador.')

    # ── Servidor web ─────────────────────────────────────────────────────

    def _snapshot(self, status):
        return {
            'state': status.state,
            'player_name': status.player_name,
            'player_score': status.player_score,
            'ai_score': status.ai_score,
            'target_score': status.target_score,
            'elapsed': status.elapsed,
            'time_limit': status.time_limit,
            'state_remaining': status.state_remaining,
            'player_won': status.player_won,
            'ranked': status.ranked,
            'rank_position': status.rank_position,
            'last_scorer': status.last_scorer,
            'difficulty': status.difficulty,
            'scores': self._scores,
        }

    def _start_web_server(self):
        threading.Thread(target=self._run_web_server, daemon=True).start()

    def _run_web_server(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        app = web.Application()
        app.router.add_get('/', self._serve('tv.html'))
        app.router.add_get('/operador', self._serve('operator.html'))
        app.router.add_get('/ws', self._websocket_handler)

        # Versões da tela da TV em avaliação. Existem só para escolher o layout
        # com o jogo rodando de verdade; a escolhida vira o tv.html e estas
        # rotas saem daqui.
        for index in (1, 2, 3, 4):
            app.router.add_get(f'/tv{index}', self._serve(f'tv_v{index}.html'))

        runner = web.AppRunner(app)
        self._loop.run_until_complete(runner.setup())

        site = web.TCPSite(runner, '0.0.0.0', self.port)

        try:
            self._loop.run_until_complete(site.start())
        except OSError as exc:
            self.get_logger().fatal(
                f'Não consegui abrir a porta {self.port}: {exc}\n'
                f'Provavelmente já há um árbitro rodando.')
            os._exit(1)

        self._loop.run_forever()

    def _serve(self, filename):
        async def handler(request):
            share = get_package_share_directory('game_master')
            return web.FileResponse(os.path.join(share, 'web', filename))
        return handler

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
                        self.get_logger().warn(f'Comando inválido: {exc}')
        finally:
            self._ws_clients.discard(ws)

        return ws

    def _broadcast(self, payload):
        if not self._ws_clients or self._loop is None:
            return

        text = json.dumps(payload)

        async def send_all():
            dead = []
            for ws in list(self._ws_clients):
                try:
                    await ws.send_str(text)
                except (ConnectionError, RuntimeError):
                    dead.append(ws)
            for ws in dead:
                self._ws_clients.discard(ws)

        asyncio.run_coroutine_threadsafe(send_all(), self._loop)


def main(args=None):
    rclpy.init(args=args)
    node = GameMaster()

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
