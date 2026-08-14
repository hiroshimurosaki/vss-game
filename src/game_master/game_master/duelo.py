"""Regras do duelo de revezamento: um robô só, dois motoristas.

Sem ROS, sem I/O, sem relógio próprio — mesma disciplina do `rules.py`. O tempo
entra por parâmetro (`now`), então dá para simular uma tarde de feira em
milissegundos e conferir que as regras fecham.

## Por que este modo existe

O formato normal precisa de dois robôs no campo. Quando só há um, não dá para
ter duelo simultâneo — mas dá para ter duelo **alternado**: o visitante dirige,
faz o gol, e então o mesmo robô passa para a IA, que faz o mesmo percurso. O que
se compara é o tempo de cada um.

O adversário sai do campo e vai para o relógio. O robô é o mesmo, o campo é o
mesmo, a bola parte do mesmo lugar — então a única variável que sobra é quem
está dirigindo, que é exatamente o que a feira quer mostrar.

## O formato

- **Turno**: um motorista, a bola no centro, o robô na marca. Acaba com gol ou
  no teto de tempo (`turn_limit`).
- **Round**: um turno do jogador e um do Franky. Ganha quem levou menos tempo.
  Quem estourou o teto sem marcar perde o round; se os dois estouraram, o round
  fica empatado e ninguém pontua.
- **Partida**: melhor de três (`rounds_to_win = 2`), com teto de `max_rounds`
  para o caso patológico de empates em série.

## As duas invariantes que sustentam a comparação

**1. Os dois atacam o MESMO gol** — o da direita, que é o que o `brain` já
ataca sem nenhuma mudança. Se cada um atacasse um lado, qualquer assimetria do
campo (iluminação, desnível, viés de uma roda) entraria direto na comparação e
ninguém conseguiria mais dizer se o Franky ganhou por ser melhor.

**2. Todo turno começa do mesmo estado** — robô na marca, bola no centro. Por
isso o `PREPARO` só libera a contagem quando o árbitro *vê* as duas coisas no
lugar (`set_ready`), e não quando um cronômetro qualquer expirou. Um turno que
começa com a bola já perto do gol não é comparável com nada.

O teto `prep_max` existe porque a fila é mais importante que a pureza: se a
visão perder a bola ou alguém esbarrar no robô, o turno começa assim mesmo, e o
operador que decida se anula. Bloquear para sempre esperando a condição ideal é
o único desfecho que não pode acontecer num estande.
"""

import math
from dataclasses import dataclass, field as dc_field

from .rules import IA, JOGADOR, format_time, insert_score  # noqa: F401


IDLE = 'IDLE'
REGISTRO = 'REGISTRO'
PREPARO = 'PREPARO'      # o Franky recoloca o robô; a bola volta para o centro
CONTAGEM = 'CONTAGEM'    # 3, 2, 1 — ninguém dirige
TURNO = 'TURNO'          # o cronômetro do motorista da vez está correndo
GOL = 'GOL'              # comemoração do gol que fechou o turno
ROUND = 'ROUND'          # anúncio do round: seu tempo x o do Franky
FIM = 'FIM'
PAUSA = 'PAUSA'

EMPATE = 'EMPATE'


@dataclass
class Config:
    rounds_to_win: int = 2       # melhor de três
    max_rounds: int = 5          # teto duro, para empate em série não travar
    turn_limit: float = 30.0     # teto de um turno, em segundos
    countdown: float = 3.0
    goal_pause: float = 3.0      # comemoração depois do gol do turno
    round_hold: float = 5.0      # quanto o anúncio do round fica no ar
    result_hold: float = 12.0
    prep_min: float = 3.0        # piso do preparo, mesmo com tudo já pronto
    prep_max: float = 20.0       # teto: começa assim mesmo, fila não espera
    max_name: int = 14


@dataclass
class Round:
    number: int = 1
    player_time: float = float('nan')
    ai_time: float = float('nan')
    player_scored: bool = False
    ai_scored: bool = False
    winner: str = ''             # JOGADOR, IA ou EMPATE


@dataclass
class Match:
    state: str = IDLE
    player_name: str = ''

    player_rounds: int = 0
    ai_rounds: int = 0

    round_number: int = 1
    driver: str = JOGADOR        # de quem é o turno atual (ou o próximo)

    elapsed: float = 0.0         # cronômetro do turno corrente
    started_at: float = 0.0
    state_until: float = 0.0

    # O round em construção. Vira uma entrada de `rounds` quando fecha.
    current: Round = dc_field(default_factory=Round)
    rounds: list = dc_field(default_factory=list)

    robot_ready: bool = False
    ball_ready: bool = False
    prep_started: float = 0.0

    # Soma dos turnos do jogador — é o que vai para o ranking.
    player_total: float = 0.0

    player_won: bool = False
    ranked: bool = False
    rank_position: int = 0
    final_time: float = 0.0

    _paused_from: str = ''
    _pause_started: float = 0.0


@dataclass
class Engine:
    """Máquina de estados do duelo alternado."""

    config: Config = dc_field(default_factory=Config)
    match: Match = dc_field(default_factory=Match)

    # Mesma trava do modo normal: a bola parada dentro do gol dispararia gol a
    # cada quadro. Rearma quando ela volta para o campo.
    _armed: bool = True
    _goal_lockout_until: float = 0.0

    # ── Transições vindas do operador ────────────────────────────────────

    def begin_registration(self, now):
        self.match = Match(state=REGISTRO)
        return self.match

    def start(self, now, player_name):
        name = (player_name or '').strip()[:self.config.max_name]

        if not name:
            name = 'VISITANTE'

        self.match = Match(state=IDLE, player_name=name)
        self._begin_prep(now, JOGADOR)
        return self.match

    def abort(self, now):
        self.match = Match(state=IDLE)
        return self.match

    def toggle_pause(self, now):
        m = self.match

        if m.state == PAUSA:
            paused_for = now - m._pause_started

            # O tempo parado não conta contra ninguém.
            m.started_at += paused_for
            m.state_until += paused_for
            m.prep_started += paused_for
            m.state = m._paused_from
            m._paused_from = ''

        elif m.state in (PREPARO, CONTAGEM, TURNO, GOL, ROUND):
            m._paused_from = m.state
            m._pause_started = now
            m.state = PAUSA

        return m

    def force_goal(self, now, scorer=None):
        """Gol marcado na mão pelo árbitro.

        O `scorer` é ignorado de propósito: no duelo o gol é sempre de quem está
        com o volante. Deixar o operador escolher só criaria a chance de creditar
        o turno errado no meio da correria.
        """
        if self.match.state != TURNO:
            return self.match

        return self._end_turn(now, scored=True)

    def skip_turn(self, now):
        """Encerra o turno atual sem gol. Para quando a bola sai e não volta."""
        if self.match.state != TURNO:
            return self.match

        return self._end_turn(now, scored=False)

    # ── Sinais do campo ──────────────────────────────────────────────────

    def set_ready(self, robot_ready, ball_ready):
        """O árbitro olhando o campo: o robô está na marca? a bola no centro?

        Só tem efeito no PREPARO — durante o turno o robô e a bola andam, e não
        faria sentido nenhum ficar reavaliando.
        """
        self.match.robot_ready = bool(robot_ready)
        self.match.ball_ready = bool(ball_ready)
        return self.match

    def on_ball(self, now, ball_x, ball_y, half_length, goal_half, margin=0.0):
        """Detecta o gol pela posição da bola. Devolve o motorista, ou None.

        Só o gol da DIREITA conta, e conta para quem estiver dirigindo. A bola
        entrando na esquerda não pune nem premia: é o gol contra, que aqui é só
        tempo perdido — o turno continua e alguém repõe a bola.
        """
        in_left = ball_x < -half_length + margin and abs(ball_y) <= goal_half
        in_right = ball_x > half_length - margin and abs(ball_y) <= goal_half

        if not in_left and not in_right:
            self._armed = True

        if self.match.state != TURNO or not in_right:
            return None

        if not self._armed or now < self._goal_lockout_until:
            return None

        self._armed = False
        driver = self.match.driver
        self._end_turn(now, scored=True)
        return driver

    def tick(self, now):
        m = self.match

        if m.state == PREPARO:
            pronto = m.robot_ready and m.ball_ready
            venceu_piso = now >= m.state_until
            estourou_teto = now >= m.prep_started + self.config.prep_max

            if (pronto and venceu_piso) or estourou_teto:
                m.state = CONTAGEM
                m.state_until = now + self.config.countdown

        elif m.state == CONTAGEM:
            if now >= m.state_until:
                m.state = TURNO
                m.started_at = now
                m.elapsed = 0.0

        elif m.state == TURNO:
            m.elapsed = now - m.started_at

            if m.elapsed >= self.config.turn_limit:
                self._end_turn(now, scored=False)

        elif m.state == GOL:
            if now >= m.state_until:
                self._after_turn(now)

        elif m.state == ROUND:
            if now >= m.state_until:
                self._after_round(now)

        elif m.state == FIM:
            if now >= m.state_until:
                m.state = IDLE

        return m

    def state_remaining(self, now):
        m = self.match

        if m.state in (CONTAGEM, GOL, ROUND, FIM):
            return max(0.0, m.state_until - now)

        if m.state == PREPARO:
            return max(0.0, m.prep_started + self.config.prep_max - now)

        if m.state == TURNO:
            return max(0.0, self.config.turn_limit - m.elapsed)

        return 0.0

    # ── Quem dirige o robô, agora ────────────────────────────────────────

    def joy_source(self):
        """Qual fonte de Joy pode chegar no robô neste instante.

        É esta função que impede o acidente clássico do projeto — dois
        produtores no mesmo `/joy_N` — porque só existe UMA resposta por
        instante, e ela é tomada aqui, num lugar só.

        No PREPARO quem dirige é a IA mesmo quando o turno seguinte é do
        visitante: é ela que leva o robô de volta à marca. Na CONTAGEM ninguém
        dirige, senão dá para largar antes do apito.
        """
        m = self.match

        if m.state == PREPARO:
            return IA

        if m.state == TURNO:
            return m.driver

        return ''

    def ai_should_go_home(self):
        return self.match.state == PREPARO

    # ── Internos ─────────────────────────────────────────────────────────

    def _begin_prep(self, now, driver):
        m = self.match
        m.driver = driver
        m.state = PREPARO
        m.prep_started = now
        m.state_until = now + self.config.prep_min
        m.elapsed = 0.0
        m.robot_ready = False
        m.ball_ready = False

    def _end_turn(self, now, scored):
        """Fecha o turno do motorista da vez e guarda o tempo dele."""
        m = self.match
        limit = self.config.turn_limit

        # Sem gol o tempo é o teto: é o que mantém os números comparáveis. Um
        # turno sem marcar não pode valer "30 s" num round e "12 s" em outro só
        # porque o operador cortou antes.
        elapsed = min(now - m.started_at, limit) if scored else limit
        m.elapsed = elapsed

        if m.driver == JOGADOR:
            m.current.player_time = elapsed
            m.current.player_scored = scored
            m.player_total += elapsed
        else:
            m.current.ai_time = elapsed
            m.current.ai_scored = scored

        if scored:
            m.state = GOL
            m.state_until = now + self.config.goal_pause
            self._goal_lockout_until = now + self.config.goal_pause + 0.5
        else:
            # Nada a comemorar: segue direto.
            self._after_turn(now)

        return m

    def _after_turn(self, now):
        m = self.match

        if m.driver == JOGADOR:
            self._begin_prep(now, IA)
        else:
            self._close_round(now)

    def _close_round(self, now):
        m = self.match
        r = m.current
        r.number = m.round_number

        if r.player_scored and r.ai_scored:
            r.winner = JOGADOR if r.player_time < r.ai_time else IA
        elif r.player_scored:
            r.winner = JOGADOR
        elif r.ai_scored:
            r.winner = IA
        else:
            r.winner = EMPATE

        if r.winner == JOGADOR:
            m.player_rounds += 1
        elif r.winner == IA:
            m.ai_rounds += 1

        m.rounds.append(r)
        m.state = ROUND
        m.state_until = now + self.config.round_hold

    def _after_round(self, now):
        m = self.match
        alvo = self.config.rounds_to_win

        acabou = (m.player_rounds >= alvo
                  or m.ai_rounds >= alvo
                  or m.round_number >= self.config.max_rounds)

        if acabou:
            self._finish(now)
            return

        m.round_number += 1
        m.current = Round(number=m.round_number)
        self._begin_prep(now, JOGADOR)

    def _finish(self, now):
        m = self.match
        m.state = FIM
        m.state_until = now + self.config.result_hold
        m.final_time = m.player_total
        m.player_won = m.player_rounds > m.ai_rounds
        return m


def round_label(round_):
    """Uma linha de anúncio para a TV, já resolvida aqui."""
    you = format_time(round_.player_time) if round_.player_scored else '—'
    ai = format_time(round_.ai_time) if round_.ai_scored else '—'
    return f'Round {round_.number}: você {you} × Franky {ai}'


def is_nan(value):
    return value is None or math.isnan(value)
