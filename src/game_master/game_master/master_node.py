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
import math
import os
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from shared_interfaces.msg import GameData, GameStatus, HighScore, HighScoreList

from aiohttp import web, WSMsgType
from ament_index_python.packages import get_package_share_directory

from . import duelo, rules, x1


def _num(value):
    """NaN vira None antes de virar JSON.

    `json.dumps` escreve `NaN` sem reclamar, mas `JSON.parse` do navegador
    rejeita — e o sintoma é a TV congelando sem erro nenhum no log do árbitro,
    porque quem quebra é o outro lado do WebSocket. Um turno que ainda não
    aconteceu tem tempo NaN, então isto acontece já no primeiro round.
    """
    if value is None or math.isnan(value):
        return None
    return round(float(value), 2)


class GameMaster(Node):

    def __init__(self):
        super().__init__('game_master')

        self.declare_parameter('port', 8090)

        # 'classico' = dois robôs, partida simultânea contra a IA (rules.py).
        # 'duelo'    = um robô só, turnos alternados contra a IA (duelo.py).
        # 'x1'       = dois robôs, duas PESSOAS, melhor de três (x1.py). É o
        #              único que roda sem câmera: sem visão não há IA, mas dois
        #              controles e um operador com o botão de gol bastam.
        self.declare_parameter('mode', 'classico')
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

        # ── Do duelo e do X1 (os dois contam em rounds) ──────────────────
        self.declare_parameter('turn_limit', 30.0)
        self.declare_parameter('rounds_to_win', 2)
        self.declare_parameter('max_rounds', 5)
        self.declare_parameter('round_hold', 5.0)
        self.declare_parameter('prep_min', 3.0)
        self.declare_parameter('prep_max', 20.0)

        # ── Só do modo X1 ────────────────────────────────────────────────
        # Teto de um round. Bem maior que o `turn_limit` do duelo porque aqui
        # são duas pessoas disputando a mesma bola, e não um percurso solo: dá
        # empate por travamento com muito mais facilidade.
        self.declare_parameter('round_limit', 90.0)

        # A marca de onde todo turno começa, e as tolerâncias com que o árbitro
        # aceita que robô e bola estão no lugar. Precisam bater com o home_x/y
        # do ai_player, senão a IA para num ponto que o árbitro não aceita e o
        # preparo sempre vai até o teto.
        self.declare_parameter('home_x', -99.0)
        self.declare_parameter('home_y', 0.0)
        # Qual robô é o do duelo. Um só em campo, e é o robô da IA que o
        # visitante toma emprestado — daí o 0, a convenção do projeto inteiro.
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('robot_ready_radius', 0.12)
        self.declare_parameter('ball_ready_radius', 0.15)

        self._mode = str(self.get_parameter('mode').value).lower()
        self._duelo = self._mode == 'duelo'
        self._x1 = self._mode == 'x1'

        # Tempo de duelo, tempo de partida clássica e placar de X1 não são a
        # mesma grandeza — um é soma de turnos, o outro é uma partida inteira, o
        # terceiro tem DOIS nomes por linha. Misturar na mesma tabela produziria
        # uma lista que não significa nada, então cada modo tem o seu arquivo.
        nome = ('highscores_x1.json' if self._x1
                else 'highscores_duelo.json' if self._duelo
                else 'highscores.json')
        default_scores = os.path.join(os.path.expanduser('~'), '.vss-game', nome)
        self.declare_parameter('scores_file', default_scores)

        if self._x1:
            self.engine = x1.Engine(config=x1.Config(
                rounds_to_win=int(self.get_parameter('rounds_to_win').value),
                max_rounds=int(self.get_parameter('max_rounds').value),
                round_limit=float(self.get_parameter('round_limit').value),
                countdown=float(self.get_parameter('countdown').value),
                goal_pause=float(self.get_parameter('goal_pause').value),
                round_hold=float(self.get_parameter('round_hold').value),
                result_hold=float(self.get_parameter('result_hold').value),
            ))
        elif self._duelo:
            self.engine = duelo.Engine(config=duelo.Config(
                rounds_to_win=int(self.get_parameter('rounds_to_win').value),
                max_rounds=int(self.get_parameter('max_rounds').value),
                turn_limit=float(self.get_parameter('turn_limit').value),
                countdown=float(self.get_parameter('countdown').value),
                goal_pause=float(self.get_parameter('goal_pause').value),
                round_hold=float(self.get_parameter('round_hold').value),
                result_hold=float(self.get_parameter('result_hold').value),
                prep_min=float(self.get_parameter('prep_min').value),
                prep_max=float(self.get_parameter('prep_max').value),
            ))
        else:
            self.engine = rules.Engine(config=rules.Config(
                target_score=int(self.get_parameter('target_score').value),
                time_limit=float(self.get_parameter('time_limit').value),
                countdown=float(self.get_parameter('countdown').value),
                goal_pause=float(self.get_parameter('goal_pause').value),
                result_hold=float(self.get_parameter('result_hold').value),
            ))

        self._half_length = float(self.get_parameter('field_length').value) / 2.0
        self._goal_half = float(self.get_parameter('goal_width').value) / 2.0
        self._goal_margin = float(self.get_parameter('goal_margin').value)
        self._difficulty = str(self.get_parameter('difficulty').value).upper()

        home_x = float(self.get_parameter('home_x').value)
        if home_x <= -99.0:
            home_x = -self._half_length * 0.55

        self._home = (home_x, float(self.get_parameter('home_y').value))
        self._robot_id = int(self.get_parameter('robot_id').value)
        self._robot_radius = float(self.get_parameter('robot_ready_radius').value)
        self._ball_radius = float(self.get_parameter('ball_ready_radius').value)

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
        self._ai_home_pub = self.create_publisher(Bool, '/ai/home', 10)
        self._joy_source_pub = self.create_publisher(String, '/game/joy_source', 10)
        self._difficulty_pub = self.create_publisher(String, '/ai/difficulty', 10)
        self._sim_reset_pub = self.create_publisher(Empty, '/sim/reset', 10)
        self._sim_ball_pub = self.create_publisher(Empty, '/sim/reset_ball', 10)

        self.create_timer(1.0 / 30.0, self._tick)

        self.port = int(self.get_parameter('port').value)
        self._ws_clients = set()
        self._loop = None
        self._start_web_server()

        self.get_logger().info(
            f'Árbitro no ar. Modo: {self._mode}.\n'
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
            if self._duelo:
                self.engine.set_ready(*self._ready_from(msg))

            scorer = self.engine.on_ball(
                self._now(), msg.ball.x, msg.ball.y,
                self._half_length, self._goal_half, self._goal_margin)

        if not scorer:
            return

        m = self.engine.match

        if self._x1:
            self.get_logger().info(
                f'GOL do {m.name(scorer)} em {x1.format_time(m.elapsed)} | '
                f'round {m.round_number}: {m.rounds_a} x {m.rounds_b}')
        elif self._duelo:
            self.get_logger().info(
                f'GOL do {scorer} em {duelo.format_time(m.elapsed)} | '
                f'round {m.round_number}: {m.player_rounds} x {m.ai_rounds}')
        else:
            self.get_logger().info(
                f'GOL do {scorer} | {m.player_score} x {m.ai_score} | '
                f'{rules.format_time(m.elapsed)}')

    def _ready_from(self, msg):
        """O árbitro olhando o campo: robô na marca? bola no centro?

        É medido aqui, e não perguntado à IA, porque quem decide se o turno pode
        começar é quem enxerga o campo. A IA pode achar que chegou e ter parado
        20 cm antes por causa de uma roda — e o turno começaria torto.
        """
        home_x, home_y = self._home

        robot_ok = False
        for robot in msg.robots:
            if robot.id != self._robot_id:
                continue
            robot_ok = math.hypot(robot.x - home_x,
                                  robot.y - home_y) <= self._robot_radius
            break

        ball_ok = math.hypot(msg.ball.x, msg.ball.y) <= self._ball_radius

        return robot_ok, ball_ok

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

        if self._x1:
            # No X1 não existe IA. Publicar False sempre não é redundância: se
            # alguém subir o `ai_player` por engano num terminal solto, ele
            # começaria a empurrar a bola no meio da partida das duas pessoas, e
            # ninguém olhando o campo entenderia de onde vem o robô possuído.
            enabled.data = False

        elif self._duelo:
            # No duelo há três respostas, não duas: a IA está jogando o turno
            # dela, está levando o robô de volta à marca, ou está fora do
            # volante. As duas primeiras exigem a IA liberada.
            fonte = self.engine.joy_source()
            enabled.data = (fonte == duelo.IA)

            home = Bool()
            home.data = self.engine.ai_should_go_home()
            self._ai_home_pub.publish(home)

            source = String()
            source.data = fonte
            self._joy_source_pub.publish(source)
        else:
            enabled.data = (match.state == rules.JOGANDO)

        self._ai_enabled_pub.publish(enabled)

        self._broadcast(self._snapshot(status))

    def _on_state_change(self, old, new):
        # Recolocar no início da contagem e depois de cada gol. No simulador
        # isso teletransporta; no campo real é um pedido para o operador — que
        # tem a duração do goal_pause para fazer.
        #
        # No duelo o momento é outro: o pedido sai no PREPARO, que é o estado
        # que existe justamente para isso e que só termina quando o árbitro vê
        # robô e bola no lugar. Mandar de novo na CONTAGEM desfaria a
        # verificação que acabou de passar.
        #
        # O X1 cai no ramo do clássico de propósito: os dois têm CONTAGEM e GOL
        # com o mesmo significado — hora de recolocar os dois robôs e a bola.
        if self._duelo:
            if new == duelo.PREPARO:
                self._sim_ball_pub.publish(Empty())
        elif new in (rules.CONTAGEM, rules.GOL):
            self._sim_reset_pub.publish(Empty())

        if new == rules.FIM:
            self._commit_result()

    def _commit_result(self):
        """Fecha a partida: grava no ranking se o jogador venceu."""
        m = self.engine.match

        if self._x1:
            self._commit_x1(m)
            return

        if not m.player_won:
            placar = (f'{m.player_rounds}x{m.ai_rounds} rounds' if self._duelo
                      else f'{m.player_score}x{m.ai_score}')
            self.get_logger().info(
                f'{m.player_name} não venceu ({placar}). Fora do ranking.')
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

    def _commit_x1(self, m):
        """Fecha a partida de X1: uma linha com os DOIS nomes no placar.

        Diferente dos outros modos, o que entra não é "o tempo de quem venceu"
        e sim o confronto inteiro — é o que permite a tela do campeonato mostrar
        `A (2) 12.4 × (1) 18.9 B`. Partida empatada não entra; quem decide isso
        é o `insert_match`, não este método, para a regra morar junto do resto
        das regras.
        """
        record = x1.match_record(m, datetime.now().strftime('%Y-%m-%d %H:%M'))

        self._scores, position = x1.insert_match(self._scores, record)

        m.ranked = position > 0
        m.rank_position = position

        self._save_scores()
        self._publish_scores()

        if m.winner == x1.EMPATE:
            self.get_logger().info(
                f'{m.name_a} {m.rounds_a} x {m.rounds_b} {m.name_b} — '
                f'empate. Fora do placar.')
            return

        self.get_logger().info(
            f'{m.name(m.winner)} venceu {m.rounds_a} x {m.rounds_b} em '
            f'{x1.format_time(x1.winning_time(record))} '
            + (f'-> {position}º do dia!' if position else '-> fora do top 10'))

    def _build_status(self, now, match):
        status = GameStatus()
        status.state = match.state
        status.player_name = match.player_name

        if self._x1:
            # O X1 tem DOIS nomes e a GameStatus só tem um campo de nome. O que
            # vai nele é o lado A; o lado B viaja no WebSocket, junto do resto do
            # detalhe. Encher a mensagem de campos novos obrigaria a recompilar o
            # shared_interfaces — e o C++ inteiro junto — só para a tela mostrar
            # mais uma coisa.
            status.player_name = match.name_a
            status.player_score = match.rounds_a
            status.ai_score = match.rounds_b
            status.target_score = self.engine.config.rounds_to_win
            status.time_limit = self.engine.config.round_limit
            status.last_scorer = match.last_scorer

        elif self._duelo:
            # O duelo é contado em rounds, não em gols. Cabe na mesma mensagem
            # sem campo novo: "quantos rounds cada um levou" ocupa exatamente o
            # lugar de "quantos gols cada um fez", e o cronômetro é o do turno
            # corrente. Quem quiser o detalhe (tempos por round) lê o WebSocket,
            # que é como a TV é alimentada.
            status.player_score = match.player_rounds
            status.ai_score = match.ai_rounds
            status.target_score = self.engine.config.rounds_to_win
            status.time_limit = self.engine.config.turn_limit
            status.last_scorer = match.driver
        else:
            status.player_score = match.player_score
            status.ai_score = match.ai_score
            status.target_score = self.engine.config.target_score
            status.time_limit = self.engine.config.time_limit
            status.last_scorer = match.last_scorer

        status.elapsed = match.elapsed
        status.state_remaining = self.engine.state_remaining(now)

        # `player_won` é a pergunta "o visitante ganhou da máquina?", que no X1
        # não existe: os dois lados são visitantes. Fica False, e quem quer saber
        # o vencedor lê `x1.winner` no WebSocket. Responder `A venceu` aqui
        # faria a tela do campeão depender de qual lado a pessoa calhou de pegar.
        status.player_won = False if self._x1 else match.player_won

        status.ranked = match.ranked
        status.rank_position = match.rank_position
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

            if self._x1:
                # A HighScore tem um nome e um tempo; a linha do X1 tem dois de
                # cada. Achatar é honesto porque este tópico existe para quem
                # está de fora (log, painel de diagnóstico), não para a tela do
                # campeonato — essa lê o registro inteiro pelo WebSocket.
                entry.name = f"{item.get('name_a', '?')} × {item.get('name_b', '?')}"
                entry.time = float(x1.winning_time(item))
                entry.difficulty = f"{item.get('score_a', 0)}x{item.get('score_b', 0)}"
            else:
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
                if self._x1:
                    self.engine.start(now, data.get('name_a', ''),
                                      data.get('name_b', ''))
                    m = self.engine.match
                    self.get_logger().info(
                        f'X1 iniciado: {m.name_a} × {m.name_b}')
                else:
                    self.engine.start(now, data.get('name', ''))
                    self.get_logger().info(
                        f'Partida iniciada: {self.engine.match.player_name}')

            elif kind == 'abort':
                self.engine.abort(now)

            elif kind == 'pause':
                self.engine.toggle_pause(now)

            elif kind == 'goal':
                if self._x1:
                    # Sem câmera, ESTE é o caminho normal do gol no X1 — não o
                    # seguro de quando a visão falha. O operador é a detecção.
                    scorer = x1.A if data.get('who') == 'a' else x1.B
                    self.engine.force_goal(now, scorer)
                    self.get_logger().info(
                        f'Gol do {self.engine.match.name(scorer)}.')
                elif self._duelo:
                    # No duelo o gol é sempre de quem está com o volante — não
                    # há como o operador creditar o turno errado.
                    driver = self.engine.match.driver
                    self.engine.force_goal(now)
                    self.get_logger().info(f'Gol manual do árbitro: {driver}')
                else:
                    scorer = (rules.JOGADOR if data.get('who') == 'player'
                              else rules.IA)
                    self.engine.force_goal(now, scorer)
                    self.get_logger().info(f'Gol manual do árbitro: {scorer}')

            elif kind == 'skip' and self._duelo:
                # A bola saiu e não volta, o robô travou na parede: encerra o
                # turno sem gol em vez de obrigar a fila a esperar o teto.
                self.engine.skip_turn(now)
                self.get_logger().info('Turno encerrado sem gol pelo operador.')

            elif kind == 'skip' and self._x1:
                self.engine.skip_round(now)
                self.get_logger().info('Round encerrado sem gol pelo operador.')

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
            'mode': self._mode,
            **(self._duelo_snapshot() if self._duelo else {}),
            **({'x1': self._x1_snapshot()} if self._x1 else {}),
        }

    def _x1_snapshot(self):
        """O detalhe que só o X1 tem, aninhado em vez de espalhado.

        O duelo espalha as chaves dele na raiz por razão histórica. Aqui vai
        aninhado de propósito: o X1 tem `name_a`/`name_b`/`score_a`… e a raiz já
        tem `player_name`/`player_score`, que no X1 são o lado A. Duas grafias do
        mesmo dado na mesma altura é a receita para a tela ler a errada num
        estado e a certa em outro.
        """
        m = self.engine.match
        c = self.engine.config

        return {
            'name_a': m.name_a,
            'name_b': m.name_b,
            'score_a': m.rounds_a,
            'score_b': m.rounds_b,
            'round_number': m.round_number,
            'rounds_to_win': c.rounds_to_win,
            'max_rounds': c.max_rounds,
            'round_limit': c.round_limit,
            'best_a': _num(m.best(x1.A)),
            'best_b': _num(m.best(x1.B)),
            'total_a': round(m.total(x1.A), 2),
            'total_b': round(m.total(x1.B), 2),
            'winner': m.winner,
            'rounds': [
                {
                    'number': r.number,
                    'time': _num(r.time),
                    'winner': r.winner,
                }
                for r in m.rounds
            ],
        }

    def _duelo_snapshot(self):
        """O detalhe que só o duelo tem, e que só a TV consome.

        Vai pelo WebSocket e não pela GameStatus de propósito: são dados de
        desenho (tempos por round, prontidão do campo), não contrato entre nós.
        Enfiá-los na mensagem obrigaria a recompilar o shared_interfaces — e o
        C++ inteiro junto — cada vez que a tela quisesse mostrar mais uma coisa.
        """
        m = self.engine.match
        c = self.engine.config

        return {
            'driver': m.driver,
            'round_number': m.round_number,
            'player_rounds': m.player_rounds,
            'ai_rounds': m.ai_rounds,
            'rounds_to_win': c.rounds_to_win,
            'turn_limit': c.turn_limit,
            'player_total': m.player_total,
            'robot_ready': m.robot_ready,
            'ball_ready': m.ball_ready,
            'current': {
                'player_time': _num(m.current.player_time),
                'ai_time': _num(m.current.ai_time),
                'player_scored': m.current.player_scored,
                'ai_scored': m.current.ai_scored,
            },
            'rounds': [
                {
                    'number': r.number,
                    'player_time': _num(r.player_time),
                    'ai_time': _num(r.ai_time),
                    'player_scored': r.player_scored,
                    'ai_scored': r.ai_scored,
                    'winner': r.winner,
                }
                for r in m.rounds
            ],
        }

    def _start_web_server(self):
        threading.Thread(target=self._web_thread_guard,
                         args=(self._run_web_server,), daemon=True).start()

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
                'servidor web do árbitro caiu ao subir e a interface não vai abrir:\n'
                + traceback.format_exc())
            os._exit(1)

    def _run_web_server(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        app = web.Application()
        # A raiz serve a tela DO MODO: o endereço que o operador decorou
        # continua sendo o mesmo, e é ele que vai na TV.
        app.router.add_get('/', self._serve(
            'x1.html' if self._x1 else 'duelo.html' if self._duelo
            else 'tv.html'))
        app.router.add_get('/duelo', self._serve('duelo.html'))
        app.router.add_get('/x1', self._serve('x1.html'))
        app.router.add_get('/operador', self._serve('operator.html'))
        app.router.add_get('/vss.css', self._serve('vss.css'))
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
            self.get_logger().fatal(
                f'Não consegui abrir a porta {self.port}: {exc}\n'
                f'Provavelmente já há um árbitro rodando.')
            os._exit(1)

        self._loop.run_forever()

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
            get_package_share_directory('game_master'), 'web', 'fonts')
        target = os.path.normpath(os.path.join(base, request.match_info['name']))

        if os.path.dirname(target) != os.path.normpath(base) or not os.path.isfile(target):
            raise web.HTTPNotFound()

        # O `mimetypes` do Python não conhece .woff2 e devolveria
        # application/octet-stream. Navegador aceita assim, mas declarar o tipo
        # certo é de graça e tira o palpite do caminho.
        return web.FileResponse(
            target, headers={'Content-Type': 'font/woff2',
                             'Cache-Control': 'public, max-age=86400'})

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
