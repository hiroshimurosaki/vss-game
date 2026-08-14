"""Regras do X1: dois robôs, duas pessoas, melhor de três. Sem IA.

Sem ROS, sem I/O, sem relógio próprio — mesma disciplina do `rules.py` e do
`duelo.py`. O tempo entra por parâmetro (`now`), então dá para simular um
campeonato inteiro em milissegundos e conferir que as regras fecham.

## Por que este modo existe

O formato normal precisa da visão para a IA jogar. Sem câmera não há IA — mas
os dois robôs continuam funcionando pelo rádio, e duas pessoas com dois
controles é um jogo inteiro que não depende de enxergar nada. O árbitro vira
humano: quem aperta o botão de gol é o operador, e é por isso que este modo é o
único que roda de ponta a ponta com o campo "cego".

## O formato

- **Round**: os dois no campo, bola no centro. Acaba no PRIMEIRO gol, ou no teto
  (`round_limit`). Quem fez o gol leva o round; se ninguém fez, o round é
  empatado e não pontua para ninguém.
- **Partida**: melhor de três (`rounds_to_win = 2`), com teto de `max_rounds`
  para o caso patológico de empates em série.

## Por que o round acaba no primeiro gol

Porque é o que produz TEMPO, e tempo é o que o placar do campeonato compara.
Um round que vai até 2 gols mede resistência; um round que acaba no primeiro
mede quem chegou primeiro, que é a mesma grandeza que o ranking do modo normal
já usa. Cada pessoa sai da partida com um número comparável ao de todo mundo
que jogou naquele dia: o menor tempo com que fechou um round.

## Quem ataca qual gol

**A ataca a esquerda, B ataca a direita** — a mesma convenção do `rules.py`,
onde a bola no gol da esquerda é ponto do visitante. Manter o mesmo lado
significa que o simulador, a visão e este módulo continuam concordando sem
nenhuma tradução no meio.

## O placar do campeonato

Cada partida vira UMA linha, com os dois nomes: `A (2) 12.4 × (1) 18.9 B`. O
tempo de cada pessoa é o **melhor round que ela venceu** — não a soma, não a
média. Soma pune quem jogou rounds longos que perdeu, média mistura vitória com
derrota; o melhor round é a única leitura que responde "qual foi o seu melhor
lance" sem depender do que o adversário fez.

Só partida DECIDIDA entra no placar. Um 1x1 que estourou o teto de rounds não
tem o que ranquear, e deixar entrar tornaria a lista uma mistura de critérios —
o mesmo motivo pelo qual o `rules.py` só ranqueia quem venceu.
"""

import math
from dataclasses import dataclass, field as dc_field

from .rules import format_time  # noqa: F401


IDLE = 'IDLE'
REGISTRO = 'REGISTRO'
CONTAGEM = 'CONTAGEM'    # 3, 2, 1 — ninguém dirige
JOGANDO = 'JOGANDO'      # o cronômetro do round está correndo
GOL = 'GOL'              # comemoração do gol que fechou o round
ROUND = 'ROUND'          # anúncio do round e do placar da partida
FIM = 'FIM'
PAUSA = 'PAUSA'

A = 'A'
B = 'B'
EMPATE = 'EMPATE'


@dataclass
class Config:
    rounds_to_win: int = 2       # melhor de três
    max_rounds: int = 5          # teto duro, para empate em série não travar
    round_limit: float = 90.0    # teto de um round, em segundos
    countdown: float = 3.0
    goal_pause: float = 4.0      # comemoração + tempo de recolocar os robôs
    round_hold: float = 5.0      # quanto o anúncio do round fica no ar
    result_hold: float = 15.0
    max_name: int = 14


@dataclass
class Round:
    number: int = 1
    time: float = float('nan')   # do apito ao gol, ou o teto se ninguém marcou
    winner: str = ''             # A, B ou EMPATE


@dataclass
class Match:
    state: str = IDLE
    name_a: str = ''
    name_b: str = ''

    rounds_a: int = 0
    rounds_b: int = 0

    round_number: int = 1

    elapsed: float = 0.0         # cronômetro do round corrente
    started_at: float = 0.0
    state_until: float = 0.0

    # O round em construção. Vira uma entrada de `rounds` quando fecha.
    current: Round = dc_field(default_factory=Round)
    rounds: list = dc_field(default_factory=list)

    last_scorer: str = ''        # quem fechou o último round

    winner: str = ''             # A, B ou EMPATE — só depois do FIM
    ranked: bool = False
    rank_position: int = 0

    _paused_from: str = ''
    _pause_started: float = 0.0

    # ── Leituras derivadas ───────────────────────────────────────────────
    #
    # Calculadas a partir de `rounds` em vez de mantidas em campo próprio: um
    # contador paralelo é mais uma coisa para esquecer de atualizar num
    # caminho de saída, e a lista de rounds nunca passa de cinco itens.

    def best(self, side):
        """Menor tempo entre os rounds que este lado venceu. NaN se nenhum."""
        tempos = [r.time for r in self.rounds
                  if r.winner == side and not is_nan(r.time)]
        return min(tempos) if tempos else float('nan')

    def total(self, side):
        """Soma dos tempos dos rounds que este lado venceu."""
        return sum(r.time for r in self.rounds
                   if r.winner == side and not is_nan(r.time))

    def score(self, side):
        return self.rounds_a if side == A else self.rounds_b

    def name(self, side):
        return self.name_a if side == A else self.name_b


@dataclass
class Engine:
    """Máquina de estados do X1."""

    config: Config = dc_field(default_factory=Config)
    match: Match = dc_field(default_factory=Match)

    # Mesma trava dos outros modos: a bola parada dentro do gol dispararia gol a
    # cada quadro. Rearma quando ela volta para o campo.
    _armed: bool = True
    _goal_lockout_until: float = 0.0

    # ── Transições vindas do operador ────────────────────────────────────

    def begin_registration(self, now):
        self.match = Match(state=REGISTRO)
        return self.match

    def start(self, now, name_a, name_b):
        self.match = Match(
            state=CONTAGEM,
            name_a=self._clean(name_a, 'JOGADOR A'),
            name_b=self._clean(name_b, 'JOGADOR B'),
            state_until=now + self.config.countdown,
        )
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
            m.state = m._paused_from
            m._paused_from = ''

        elif m.state in (CONTAGEM, JOGANDO, GOL, ROUND):
            m._paused_from = m.state
            m._pause_started = now
            m.state = PAUSA

        return m

    def force_goal(self, now, scorer):
        """Gol marcado na mão pelo árbitro.

        No X1 este NÃO é o caminho de exceção: é o caminho normal. Sem câmera,
        o operador é a única detecção de gol que existe.
        """
        if self.match.state != JOGANDO:
            return self.match

        if scorer not in (A, B):
            return self.match

        return self._end_round(now, scorer)

    def skip_round(self, now):
        """Encerra o round sem gol. Para quando a bola sai e não volta."""
        if self.match.state != JOGANDO:
            return self.match

        return self._end_round(now, EMPATE)

    # ── Sinais do campo ──────────────────────────────────────────────────

    def on_ball(self, now, ball_x, ball_y, half_length, goal_half, margin=0.0):
        """Detecta o gol pela posição da bola. Devolve o marcador, ou None.

        Só existe quando há visão ou simulador. No campo cego da feira este
        caminho simplesmente nunca é chamado, e o modo funciona igual.
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

        scorer = A if in_left else B if in_right else None

        if scorer is None:
            return None

        self._armed = False
        self._end_round(now, scorer)
        return scorer

    def tick(self, now):
        m = self.match

        if m.state == CONTAGEM:
            if now >= m.state_until:
                m.state = JOGANDO
                m.started_at = now
                m.elapsed = 0.0

        elif m.state == JOGANDO:
            m.elapsed = now - m.started_at

            if m.elapsed >= self.config.round_limit:
                self._end_round(now, EMPATE)

        elif m.state == GOL:
            if now >= m.state_until:
                self._announce_round(now)

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

        if m.state == JOGANDO:
            return max(0.0, self.config.round_limit - m.elapsed)

        return 0.0

    # ── Internos ─────────────────────────────────────────────────────────

    def _clean(self, raw, fallback):
        name = (raw or '').strip()[:self.config.max_name]
        return name or fallback

    def _begin_round(self, now):
        m = self.match
        m.current = Round(number=m.round_number)
        m.state = CONTAGEM
        m.state_until = now + self.config.countdown
        m.elapsed = 0.0

    def _end_round(self, now, winner):
        """Fecha o round corrente e credita quem marcou."""
        m = self.match
        limit = self.config.round_limit

        # Sem gol o tempo é o teto: é o que mantém os números comparáveis. Um
        # round sem gol não pode valer "12 s" só porque o operador cortou antes.
        elapsed = (min(now - m.started_at, limit) if winner in (A, B)
                   else limit)

        m.elapsed = elapsed

        r = m.current
        r.number = m.round_number
        r.time = elapsed
        r.winner = winner

        if winner == A:
            m.rounds_a += 1
        elif winner == B:
            m.rounds_b += 1

        m.rounds.append(r)
        m.last_scorer = winner

        if winner in (A, B):
            m.state = GOL
            m.state_until = now + self.config.goal_pause
            self._goal_lockout_until = now + self.config.goal_pause + 0.5
        else:
            # Nada a comemorar: vai direto para o anúncio do round.
            self._announce_round(now)

        return m

    def _announce_round(self, now):
        m = self.match
        m.state = ROUND
        m.state_until = now + self.config.round_hold

    def _after_round(self, now):
        m = self.match
        alvo = self.config.rounds_to_win

        acabou = (m.rounds_a >= alvo
                  or m.rounds_b >= alvo
                  or m.round_number >= self.config.max_rounds)

        if acabou:
            self._finish(now)
            return

        m.round_number += 1
        self._begin_round(now)

    def _finish(self, now):
        m = self.match
        m.state = FIM
        m.state_until = now + self.config.result_hold

        if m.rounds_a > m.rounds_b:
            m.winner = A
        elif m.rounds_b > m.rounds_a:
            m.winner = B
        else:
            m.winner = EMPATE

        return m


# ── Placar do campeonato ─────────────────────────────────────────────────

def match_record(match, date):
    """A partida virando UMA linha do placar. Só faz sentido depois do FIM."""
    return {
        'name_a': match.name_a,
        'name_b': match.name_b,
        'score_a': match.rounds_a,
        'score_b': match.rounds_b,
        'time_a': _clean_time(match.best(A)),
        'time_b': _clean_time(match.best(B)),
        'winner': match.winner,
        'date': date,
    }


def insert_match(entries, record, limit=10):
    """Insere no placar e devolve (lista_nova, posição). Posição 0 = não entrou.

    Ordenado pelo tempo do VENCEDOR, crescente: a lista responde "qual foi a
    vitória mais rápida do dia", que é a mesma pergunta do ranking do modo
    normal. Partida sem vencedor não entra — ver o cabeçalho do módulo.
    """
    if record.get('winner') not in (A, B):
        return list(entries), 0

    merged = list(entries) + [record]
    merged.sort(key=winning_time)
    merged = merged[:limit]

    for index, item in enumerate(merged):
        if item is record:
            return merged, index + 1

    return merged, 0


def winning_time(record):
    """O tempo que ordena a linha: o melhor round de quem venceu.

    Devolve infinito quando não há tempo, para a linha ir para o fim em vez de
    explodir a comparação. Não deveria acontecer — quem vence ganhou ao menos um
    round e portanto tem tempo — mas um placar lido de disco pode vir de uma
    versão anterior, e a lista da feira não pode quebrar por causa disso.
    """
    lado = 'time_a' if record.get('winner') == A else 'time_b'
    valor = record.get(lado)

    if valor is None or is_nan(valor):
        return float('inf')

    return float(valor)


def match_label(record):
    """A linha já resolvida, do jeito que vai para a tela.

    O lado B sai em tom mais escuro na tela; aqui é só o texto.
    """
    return (f"{record['name_a']} ({record['score_a']}) "
            f"{format_time(record['time_a'])} × "
            f"{format_time(record['time_b'])} "
            f"({record['score_b']}) {record['name_b']}")


def round_label(round_):
    """Uma linha de anúncio do round para a TV, já resolvida aqui."""
    if round_.winner == EMPATE:
        return f'Round {round_.number}: sem gol'

    return f'Round {round_.number}: {round_.winner} em {format_time(round_.time)}'


def _clean_time(value):
    """NaN vira None antes de virar JSON — `JSON.parse` rejeita `NaN`."""
    if value is None or is_nan(value):
        return None
    return round(float(value), 2)


def is_nan(value):
    if value is None:
        return True
    try:
        return math.isnan(float(value))
    except (TypeError, ValueError):
        return True
