"""Regras da partida. Sem ROS, sem I/O, sem relógio próprio.

O tempo entra por parâmetro (`now`), então dá para simular uma tarde inteira de
feira em milissegundos e conferir que as regras fecham.

## O formato

Primeiro a fazer 2 gols vence, com teto de tempo. O cronômetro corre do apito
até o gol que decide — é ele que vai para o ranking, ordenado do menor para o
maior. Quem perde ou estoura o tempo não entra.

Duas decisões de design que vêm da feira, não do futebol:

  - **Teto de tempo.** Fila parada é o pior inimigo de um jogo de estande. Sem
    teto, uma dupla travada em 1x1 segura vinte pessoas.
  - **Só o vencedor entra no ranking.** Faz a lista significar uma coisa só
    ("quem venceu mais rápido") em vez de misturar critérios.
"""

import math
from dataclasses import dataclass, field as dc_field


IDLE = 'IDLE'
REGISTRO = 'REGISTRO'
CONTAGEM = 'CONTAGEM'
JOGANDO = 'JOGANDO'
GOL = 'GOL'
FIM = 'FIM'
PAUSA = 'PAUSA'

JOGADOR = 'JOGADOR'
IA = 'IA'


@dataclass
class Config:
    target_score: int = 2          # primeiro a tantos gols vence
    time_limit: float = 180.0      # teto, em segundos
    countdown: float = 3.0         # duração do 3, 2, 1
    goal_pause: float = 4.0        # comemoração + tempo de recolocar os robôs
    result_hold: float = 12.0      # quanto a tela de resultado fica no ar
    max_name: int = 14


@dataclass
class Match:
    state: str = IDLE
    player_name: str = ''
    player_score: int = 0
    ai_score: int = 0

    started_at: float = 0.0
    elapsed: float = 0.0           # congela quando não está JOGANDO
    state_until: float = 0.0       # quando o estado atual expira

    last_scorer: str = ''
    player_won: bool = False
    ranked: bool = False
    rank_position: int = 0

    # Guarda o cronômetro no momento em que a partida acabou.
    final_time: float = 0.0

    _paused_from: str = ''
    _pause_started: float = 0.0


@dataclass
class Engine:
    """Máquina de estados da partida."""

    config: Config = dc_field(default_factory=Config)
    match: Match = dc_field(default_factory=Match)

    # Depois de um gol, a bola precisa sair da área antes de valer outro. Só
    # travar por tempo não basta: se ela ficar presa no fundo do gol — o que
    # acontece de verdade quando ninguém recoloca — o lockout expira e o placar
    # dispara em série.
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

        self.match = Match(
            state=CONTAGEM,
            player_name=name,
            state_until=now + self.config.countdown,
        )
        return self.match

    def abort(self, now):
        """Cancela o que estiver rolando e volta para a tela de atração."""
        self.match = Match(state=IDLE)
        return self.match

    def toggle_pause(self, now):
        m = self.match

        if m.state == PAUSA:
            resumed = m._paused_from
            paused_for = now - m._pause_started

            # O tempo parado não conta contra o jogador.
            m.started_at += paused_for
            m.state_until += paused_for
            m.state = resumed
            m._paused_from = ''

        elif m.state in (CONTAGEM, JOGANDO, GOL):
            m._paused_from = m.state
            m._pause_started = now
            m.state = PAUSA

        return m

    def force_goal(self, now, scorer):
        """Gol marcado pelo árbitro humano.

        Existe desde o primeiro dia e nunca sai: é o que mantém o jogo rodando
        quando a visão falha na feira.
        """
        if self.match.state not in (JOGANDO, GOL):
            return self.match

        return self._register_goal(now, scorer)

    # ── Transições automáticas ───────────────────────────────────────────

    def on_ball(self, now, ball_x, ball_y, half_length, goal_half, margin=0.0):
        """Detecta gol pela posição da bola. Devolve o marcador, ou None.

        A regra vale igual no simulador e no campo real, porque os dois falam a
        mesma /game_data. O árbitro é este nó, não o simulador.
        """
        in_left = ball_x < -half_length + margin and abs(ball_y) <= goal_half
        in_right = ball_x > half_length - margin and abs(ball_y) <= goal_half

        # Rearma assim que a bola volta para o campo.
        if not in_left and not in_right:
            self._armed = True

        if self.match.state != JOGANDO:
            return None

        if not self._armed or now < self._goal_lockout_until:
            return None

        # A IA defende a esquerda: bola lá dentro é ponto do jogador.
        if in_left:
            self._armed = False
            self._register_goal(now, JOGADOR)
            return JOGADOR

        if in_right:
            self._armed = False
            self._register_goal(now, IA)
            return IA

        return None

    def tick(self, now):
        """Avança o tempo. Chame sempre; ela decide se algo muda."""
        m = self.match

        if m.state == JOGANDO:
            m.elapsed = now - m.started_at

            if m.elapsed >= self.config.time_limit:
                self._finish(now, timed_out=True)

        elif m.state == CONTAGEM:
            if now >= m.state_until:
                m.state = JOGANDO
                m.started_at = now
                m.elapsed = 0.0

        elif m.state == GOL:
            if now >= m.state_until:
                if self._is_over():
                    self._finish(now)
                else:
                    # Volta a jogar de onde o cronômetro parou.
                    m.state = JOGANDO
                    m.started_at = now - m.elapsed

        elif m.state == FIM:
            if now >= m.state_until:
                m.state = IDLE

        return m

    def state_remaining(self, now):
        m = self.match

        if m.state in (CONTAGEM, GOL, FIM):
            return max(0.0, m.state_until - now)

        if m.state == JOGANDO:
            return max(0.0, self.config.time_limit - m.elapsed)

        return 0.0

    # ── Internos ─────────────────────────────────────────────────────────

    def _register_goal(self, now, scorer):
        m = self.match

        # Congela o cronômetro no instante do gol.
        m.elapsed = now - m.started_at

        if scorer == JOGADOR:
            m.player_score += 1
        else:
            m.ai_score += 1

        m.last_scorer = scorer
        m.state = GOL
        m.state_until = now + self.config.goal_pause

        # Meio segundo de carência: a bola ainda está dentro do gol e seria
        # contada de novo no próximo quadro.
        self._goal_lockout_until = now + self.config.goal_pause + 0.5

        return m

    def _is_over(self):
        m = self.match
        target = self.config.target_score
        return m.player_score >= target or m.ai_score >= target

    def _finish(self, now, timed_out=False):
        m = self.match

        m.state = FIM
        m.state_until = now + self.config.result_hold
        m.final_time = m.elapsed

        # Estourar o tempo nunca é vitória, mesmo estando na frente: o
        # cronômetro é a métrica do ranking, e premiar quem não concluiu
        # tornaria os tempos incomparáveis.
        m.player_won = (not timed_out
                        and m.player_score >= self.config.target_score)

        return m


# ── Ranking ──────────────────────────────────────────────────────────────

def insert_score(entries, name, seconds, difficulty, date, limit=10):
    """Insere no ranking e devolve (lista_nova, posição). Posição 0 = não entrou.

    Ordenado por tempo crescente: vencer rápido é o que vale.
    """
    entry = {
        'name': name,
        'time': round(seconds, 2),
        'difficulty': difficulty,
        'date': date,
    }

    merged = list(entries) + [entry]
    merged.sort(key=lambda e: e['time'])
    merged = merged[:limit]

    for index, item in enumerate(merged):
        if item is entry:
            return merged, index + 1

    return merged, 0


def format_time(seconds):
    """mm:ss.d — o formato que cabe grande na TV."""
    if seconds is None or math.isnan(seconds):
        return '--:--'

    minutes = int(seconds // 60)
    rest = seconds - minutes * 60
    return f'{minutes}:{rest:04.1f}'
