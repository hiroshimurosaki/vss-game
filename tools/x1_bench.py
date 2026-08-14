#!/usr/bin/env python3
"""Roda partidas de X1 inteiras com relógio falso, sem ROS.

    ./tools/x1_bench.py

O `x1.py` é puro de propósito — o tempo entra por parâmetro — e é isso que
permite conferir uma tarde de feira em milissegundos. Este arquivo é o teste de
regressão do modo X1, na mesma função que o `duelo_bench.py` cumpre para o
duelo: se alguém mexer nas regras e quebrar uma invariante, é aqui que aparece
antes de aparecer na frente de duas crianças com um controle na mão cada.

Cada caso confere uma coisa que custaria caro descobrir na feira:

    2x0 e 2x1          o placar de rounds, quem venceu e QUAL tempo vai ao
                       ranking (o melhor round, não a soma nem o último)
    round sem gol      que ele empata e registra o TETO, não o decorrido
    skip_round         que o corte na mão registra o mesmo teto
    empate em série    que a partida TERMINA e que ela NÃO entra no placar
    pausa              que o tempo parado não conta contra ninguém
    trava do gol       que a bola parada no gol não fecha round a cada quadro
    ordenação          que o placar sai pelo tempo do vencedor, crescente
    nomes              o fallback do nome vazio e o corte do nome comprido
    JSON               que nenhum registro carrega NaN para o navegador

Sai com código 1 se qualquer caso falhar: isto é teste, não relatório.
"""

import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'game_master'))

from game_master import x1                        # noqa: E402

HALF_LEN = 0.75
GOAL_HALF = 0.20
HOJE = '2026-08-14'

# A bola no gol da ESQUERDA é ponto de A — a mesma convenção do rules.py, e é
# por isso que este módulo, a visão e o simulador não precisam de tradução.
GOL_DE_A = (-HALF_LEN - 0.01, 0.0)
GOL_DE_B = (HALF_LEN + 0.01, 0.0)
CENTRO = (0.0, 0.0)


class Sim:
    """O motor com um relógio que a gente avança na mão."""

    def __init__(self, **cfg):
        self.e = x1.Engine(config=x1.Config(**cfg))
        self.t = 0.0

    # ── Relógio ──────────────────────────────────────────────────────────

    def avanca(self, dt, passo=0.05, bola=None):
        """Anda dt segundos em passos curtos, deixando o motor reagir.

        `bola` só existe para o caso da trava do gol: no campo cego da feira
        ninguém chama `on_ball`, e todos os outros casos rodam sem ela.
        """
        alvo = self.t + dt
        while self.t < alvo - 1e-9:
            self.t = min(self.t + passo, alvo)
            if bola is not None:
                self.e.on_ball(self.t, bola[0], bola[1], HALF_LEN, GOAL_HALF)
            self.e.tick(self.t)
        return self.e.match

    def ate(self, estado, teto=600.0, bola=None):
        """Avança até entrar no estado pedido. Estoura se nunca entrar."""
        gasto = 0.0
        while self.e.match.state != estado and gasto < teto:
            self.avanca(0.05, bola=bola)
            gasto += 0.05
        assert self.e.match.state == estado, \
            f'esperava {estado}, ficou em {self.e.match.state}'
        return self.e.match

    # ── Um round inteiro ─────────────────────────────────────────────────

    def joga_round(self, segundos, marcador):
        """Espera o apito, gasta o tempo exato e faz o gol pelo painel.

        O relógio pula direto para `started_at + segundos` em vez de somar
        passos: o tempo do round é o número que vai para o placar do dia, e um
        teste que o confere com tolerância não confere nada.

        `force_goal` é o caminho NORMAL deste modo, não o de exceção — sem
        câmera, o operador é a única detecção de gol que existe.
        """
        self.ate(x1.JOGANDO)
        self.t = self.e.match.started_at + segundos
        self.e.force_goal(self.t, marcador)
        assert self.e.match.state == x1.GOL, self.e.match.state
        return self.e.match.rounds[-1]

    def round_sem_gol(self):
        """Deixa o round estourar o teto sozinho."""
        self.ate(x1.JOGANDO)
        self.avanca(self.e.config.round_limit + 0.2)
        return self.e.match.rounds[-1]


# ── Os casos ─────────────────────────────────────────────────────────────
#
# Cada um devolve a string com os valores exatos que o runner imprime ao lado
# do PASSOU. Falha = AssertionError, que o runner transforma em FALHOU.


def caso_partida_2x0():
    """A vence os dois primeiros rounds e a partida acaba ali."""
    sim = Sim()
    sim.e.start(0.0, 'ANA', 'BRUNO')
    assert sim.e.match.state == x1.CONTAGEM, sim.e.match.state

    r1 = sim.joga_round(12.5, x1.A)
    assert r1.winner == x1.A and r1.number == 1, r1
    r2 = sim.joga_round(9.0, x1.A)
    assert r2.winner == x1.A and r2.number == 2, r2

    sim.ate(x1.FIM)
    m = sim.e.match

    assert m.winner == x1.A, m.winner
    assert (m.rounds_a, m.rounds_b) == (2, 0), (m.rounds_a, m.rounds_b)
    # O terceiro round não pode ter acontecido: dois já decidem.
    assert len(m.rounds) == 2, len(m.rounds)
    assert m.round_number == 2, m.round_number
    assert m.best(x1.A) == 9.0, m.best(x1.A)
    assert x1.is_nan(m.best(x1.B)), m.best(x1.B)

    return (f'winner={m.winner} placar={m.rounds_a}x{m.rounds_b} '
            f'rounds jogados={len(m.rounds)} best(A)={m.best(x1.A)}')


def caso_partida_2x1():
    """B leva o round 1, A leva o 2 e o 3. O tempo de A é o MENOR."""
    sim = Sim()
    sim.e.start(0.0, 'ANA', 'BRUNO')

    r1 = sim.joga_round(20.0, x1.B)
    assert r1.winner == x1.B, r1
    r2 = sim.joga_round(9.0, x1.A)     # o melhor de A
    r3 = sim.joga_round(15.0, x1.A)    # o último de A, e o PIOR dos dois
    assert (r2.winner, r3.winner) == (x1.A, x1.A), (r2, r3)

    sim.ate(x1.FIM)
    m = sim.e.match

    assert m.winner == x1.A, m.winner
    assert (m.rounds_a, m.rounds_b) == (2, 1), (m.rounds_a, m.rounds_b)
    assert len(m.rounds) == 3, len(m.rounds)

    # A invariante do modo: o tempo de cada um é o MELHOR round que ele venceu.
    # Não a soma (24.0, que puniria quem venceu dois) e não o último (15.0, que
    # dependeria da ordem em que os rounds caíram).
    #
    # Tolerância, e não igualdade exata: estes tempos saem de `now - started_at`
    # com o `now` acumulado round a round, então o terceiro round já carrega
    # ruído de ponto flutuante (15.000000000000007). Não é defeito do motor e
    # não chega em tela nenhuma — o `match_record` arredonda em duas casas antes
    # de virar JSON e o `format_time` mostra uma. Onde a igualdade exata É a
    # regra, ela continua exata: ver `caso_round_sem_gol`, cujo tempo é o
    # `round_limit` literal e não uma diferença de relógio.
    assert abs(m.best(x1.A) - 9.0) < 1e-6, m.best(x1.A)
    assert abs(m.total(x1.A) - 24.0) < 1e-6, m.total(x1.A)
    assert abs(r3.time - 15.0) < 1e-6, r3.time
    assert abs(m.best(x1.B) - 20.0) < 1e-6, m.best(x1.B)

    return (f'winner={m.winner} placar={m.rounds_a}x{m.rounds_b} '
            f'best(A)={m.best(x1.A)} (soma={m.total(x1.A)}, '
            f'último={r3.time}) best(B)={m.best(x1.B)}')


def caso_round_sem_gol():
    """Ninguém marca até o teto: empate, sem ponto, tempo = round_limit."""
    sim = Sim(round_limit=30.0)
    sim.e.start(0.0, 'ANA', 'BRUNO')

    r = sim.round_sem_gol()
    m = sim.e.match

    assert r.winner == x1.EMPATE, r.winner
    assert (m.rounds_a, m.rounds_b) == (0, 0), (m.rounds_a, m.rounds_b)
    # Igualdade EXATA, não aproximada: o teto é o que mantém os tempos dos
    # rounds sem gol comparáveis entre si e com os que tiveram gol.
    assert r.time == 30.0, r.time
    assert m.last_scorer == x1.EMPATE, m.last_scorer

    return (f'winner={r.winner} placar={m.rounds_a}x{m.rounds_b} '
            f'time={r.time} (round_limit={sim.e.config.round_limit})')


def caso_skip_round():
    """O corte na mão no meio do round também registra o teto."""
    sim = Sim(round_limit=30.0)
    sim.e.start(0.0, 'ANA', 'BRUNO')
    sim.ate(x1.JOGANDO)

    # 8 s de round e o operador corta (a bola saiu e não voltou).
    sim.t = sim.e.match.started_at + 8.0
    sim.e.skip_round(sim.t)

    m = sim.e.match
    r = m.rounds[-1]

    assert r.winner == x1.EMPATE, r.winner
    assert (m.rounds_a, m.rounds_b) == (0, 0), (m.rounds_a, m.rounds_b)
    # Se registrasse os 8 s decorridos, um round abandonado viraria o melhor
    # tempo do dia. Registra o teto.
    assert r.time == 30.0, r.time
    # Sem gol não há o que comemorar: pula o GOL e vai direto ao anúncio.
    assert m.state == x1.ROUND, m.state

    return f'decorrido=8.0 registrado={r.time} winner={r.winner} state={m.state}'


def caso_empate_em_serie():
    """Todos os rounds empatados: para no max_rounds e não entra no placar."""
    sim = Sim(round_limit=20.0, max_rounds=3)
    sim.e.start(0.0, 'ANA', 'BRUNO')

    for _ in range(3):
        sim.round_sem_gol()
        sim.ate(x1.ROUND)
        sim.avanca(sim.e.config.round_hold + 0.2)

    sim.ate(x1.FIM)
    m = sim.e.match

    assert m.round_number == 3, m.round_number
    assert len(m.rounds) == 3, len(m.rounds)
    assert (m.rounds_a, m.rounds_b) == (0, 0), (m.rounds_a, m.rounds_b)
    assert m.winner == x1.EMPATE, m.winner
    assert all(r.winner == x1.EMPATE for r in m.rounds), m.rounds

    # Partida sem vencedor não tem o que ranquear: deixar entrar tornaria a
    # lista uma mistura de critérios.
    antes = [x1.match_record(m, HOJE)]      # uma linha qualquer já no placar
    antes[0]['winner'] = x1.A               # ...essa sim, decidida
    antes[0]['time_a'] = 5.0

    depois, posicao = x1.insert_match(antes, x1.match_record(m, HOJE))

    assert posicao == 0, posicao
    assert depois == antes, (depois, antes)

    return (f'parou no round {m.round_number} winner={m.winner} '
            f'posicao={posicao} placar continua com {len(depois)} linha(s)')


def caso_pausa():
    """Pausar 30 s no meio do round não muda o tempo final dele."""
    # A referência: o mesmo round, sem pausa nenhuma.
    limpo = Sim()
    limpo.e.start(0.0, 'ANA', 'BRUNO')
    referencia = limpo.joga_round(12.0, x1.A).time

    sim = Sim()
    sim.e.start(0.0, 'ANA', 'BRUNO')
    sim.ate(x1.JOGANDO)

    sim.t = sim.e.match.started_at + 5.0
    sim.e.toggle_pause(sim.t)
    assert sim.e.match.state == x1.PAUSA, sim.e.match.state

    sim.avanca(30.0)                        # meia eternidade parado
    assert sim.e.match.state == x1.PAUSA, sim.e.match.state
    sim.e.toggle_pause(sim.t)
    assert sim.e.match.state == x1.JOGANDO, sim.e.match.state

    # O `started_at` andou junto com a pausa, então o alvo de 12 s continua
    # sendo 12 s de jogo.
    sim.t = sim.e.match.started_at + 12.0
    r = sim.e.force_goal(sim.t, x1.A).rounds[-1]

    assert abs(r.time - referencia) < 1e-6, (r.time, referencia)
    assert abs(r.time - 12.0) < 1e-6, r.time

    return (f'com pausa={r.time!r} sem pausa={referencia!r} '
            f'diferença={abs(r.time - referencia):.2e}')


def caso_trava_do_gol():
    """A bola parada dentro do gol não fecha um round a cada quadro."""
    sim = Sim()
    sim.e.start(0.0, 'ANA', 'BRUNO')
    sim.ate(x1.JOGANDO)

    sim.t = sim.e.match.started_at + 6.0
    marcadores = [sim.e.on_ball(sim.t, GOL_DE_A[0], GOL_DE_A[1],
                                HALF_LEN, GOAL_HALF)
                  for _ in range(20)]

    assert marcadores[0] == x1.A, marcadores[0]
    assert all(x is None for x in marcadores[1:]), marcadores[:5]
    assert len(sim.e.match.rounds) == 1, len(sim.e.match.rounds)

    # E ela continua sem valer quando o round SEGUINTE começa, com a bola
    # ainda parada lá dentro: quem rearma é ela voltar ao campo, não o relógio.
    sim.ate(x1.JOGANDO, bola=GOL_DE_A)
    sim.avanca(1.0, bola=GOL_DE_A)
    assert len(sim.e.match.rounds) == 1, len(sim.e.match.rounds)
    assert sim.e.match.rounds_a == 1, sim.e.match.rounds_a

    # Rearma quando a bola volta ao campo, e aí o gol seguinte vale. Vai no gol
    # da DIREITA, que de quebra confere o lado: direita é ponto de B.
    sim.e.on_ball(sim.t, CENTRO[0], CENTRO[1], HALF_LEN, GOAL_HALF)
    segundo = sim.e.on_ball(sim.t, GOL_DE_B[0], GOL_DE_B[1],
                            HALF_LEN, GOAL_HALF)

    assert segundo == x1.B, segundo
    assert len(sim.e.match.rounds) == 2, len(sim.e.match.rounds)
    assert (sim.e.match.rounds_a, sim.e.match.rounds_b) == (1, 1), \
        (sim.e.match.rounds_a, sim.e.match.rounds_b)

    fechados = sum(1 for x in marcadores if x is not None)
    return (f'{len(marcadores)} chamadas com a bola parada no gol fecharam '
            f'{fechados} round; depois de rearmar, {len(sim.e.match.rounds)}')


def _registro(nome, tempo):
    """Uma partida decidida por A, com `tempo` como o melhor round dele.

    Montada pelo `match_record` de verdade em vez de um dicionário na mão: é o
    caminho que a TV usa, e assim o teste do placar também cobre o `best`.
    """
    m = x1.Match(name_a=nome, name_b='ADVERSARIO', rounds_a=2, rounds_b=0,
                 winner=x1.A)
    m.rounds = [x1.Round(number=1, time=tempo + 7.0, winner=x1.A),
                x1.Round(number=2, time=tempo, winner=x1.A)]
    registro = x1.match_record(m, HOJE)
    assert registro['time_a'] == round(tempo, 2), registro
    return registro


def caso_ordenacao_do_placar():
    """12 partidas embaralhadas saem em ordem de tempo, cortadas em 10."""
    tempos = [23.4, 8.1, 41.0, 12.7, 5.5, 33.2, 19.9, 7.3, 27.8, 15.0,
              3.2, 38.6]

    placar = []
    for i, tempo in enumerate(tempos):
        registro = _registro(f'P{i:02d}', tempo)
        placar, posicao = x1.insert_match(placar, registro)

        # A posição devolvida é o que a TV usa para dizer "você é o 3º": se ela
        # não bater com o índice real, a tela mente sem quebrar nada.
        if posicao:
            assert placar[posicao - 1] is registro, (posicao, tempo)
        else:
            assert registro not in placar, tempo

    saiu = [r['time_a'] for r in placar]
    esperado = sorted(tempos)[:10]

    assert len(placar) == 10, len(placar)
    assert saiu == esperado, (saiu, esperado)
    assert saiu == sorted(saiu), saiu
    assert [x1.winning_time(r) for r in placar] == esperado, saiu
    # Os dois mais lentos ficaram de fora.
    assert 38.6 not in saiu and 41.0 not in saiu, saiu

    return f'{len(placar)} linhas, tempos {saiu}'


def caso_nomes():
    """Nome vazio cai no padrão; nome comprido é cortado em max_name."""
    sim = Sim()
    m = sim.e.start(0.0, '', '')

    assert m.name_a == 'JOGADOR A', repr(m.name_a)
    assert m.name_b == 'JOGADOR B', repr(m.name_b)
    assert m.name(x1.A) == 'JOGADOR A', m.name(x1.A)

    # Espaço em branco é o mesmo que vazio — o operador aperta espaço sem
    # querer no teclado do painel.
    branco = sim.e.start(0.0, '   ', ' ')
    assert (branco.name_a, branco.name_b) == ('JOGADOR A', 'JOGADOR B'), branco

    comprido = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ1234'      # 30 letras
    assert len(comprido) == 30, len(comprido)
    cortado = sim.e.start(0.0, comprido, comprido)

    assert len(cortado.name_a) == 14, len(cortado.name_a)
    assert cortado.name_a == comprido[:14], cortado.name_a
    assert cortado.name_a == 'ABCDEFGHIJKLMN', cortado.name_a

    return (f"vazio -> {m.name_a!r}/{m.name_b!r}; "
            f"30 letras -> {cortado.name_a!r} ({len(cortado.name_a)})")


def caso_sem_nan_no_json():
    """Nenhum registro pode carregar NaN para o outro lado do WebSocket.

    `JSON.parse` do navegador rejeita `NaN`, e o sintoma é a TV CONGELANDO sem
    uma linha de erro no log do árbitro — quem quebra é o outro lado. O
    `_clean_time` troca NaN por None; este caso é a cerca em volta dele.
    """
    registros = []

    # 1) Partida decidida 2x0: o lado B nunca venceu um round, então best(B)
    #    é NaN. É o caso mais comum da feira, não uma exceção.
    a = Sim()
    a.e.start(0.0, 'ANA', 'BRUNO')
    a.joga_round(12.5, x1.A)
    a.joga_round(9.0, x1.A)
    a.ate(x1.FIM)
    assert x1.is_nan(a.e.match.best(x1.B)), a.e.match.best(x1.B)
    registros.append(x1.match_record(a.e.match, HOJE))

    # 2) Empate em série: os DOIS lados sem tempo.
    b = Sim(round_limit=20.0, max_rounds=3)
    b.e.start(0.0, 'ANA', 'BRUNO')
    for _ in range(3):
        b.round_sem_gol()
        b.ate(x1.ROUND)
        b.avanca(b.e.config.round_hold + 0.2)
    b.ate(x1.FIM)
    registros.append(x1.match_record(b.e.match, HOJE))

    # 3) Partida que nem começou: tudo zerado e os tempos ainda NaN.
    registros.append(x1.match_record(x1.Match(), HOJE))

    # 4) Uma partida normal 2x1, para o caminho feliz não passar despercebido.
    c = Sim()
    c.e.start(0.0, 'ANA', 'BRUNO')
    c.joga_round(20.0, x1.B)
    c.joga_round(9.0, x1.A)
    c.joga_round(15.0, x1.A)
    c.ate(x1.FIM)
    registros.append(x1.match_record(c.e.match, HOJE))

    for i, registro in enumerate(registros):
        # allow_nan=False é exatamente o que o json do navegador faz.
        texto = json.dumps(registro, allow_nan=False)
        assert 'NaN' not in texto, (i, texto)
        assert 'Infinity' not in texto, (i, texto)

    # E o snapshot inteiro, que é o que sai de fato pelo WebSocket.
    snapshot = json.dumps({'mode': 'x1', 'scores': registros}, allow_nan=False)

    return (f'{len(registros)} registros e o snapshot ({len(snapshot)} bytes) '
            f'passaram no json.dumps(allow_nan=False)')


CASOS = (
    caso_partida_2x0,
    caso_partida_2x1,
    caso_round_sem_gol,
    caso_skip_round,
    caso_empate_em_serie,
    caso_pausa,
    caso_trava_do_gol,
    caso_ordenacao_do_placar,
    caso_nomes,
    caso_sem_nan_no_json,
)


def main():
    print('bancada do X1 — regras de game_master/x1.py, sem ROS\n')

    passaram = 0
    for caso in CASOS:
        nome = caso.__name__.replace('caso_', '')
        try:
            detalhe = caso()
        except AssertionError as erro:
            print(f'  FALHOU  {nome:22s} {erro}')
        except Exception as erro:                       # noqa: BLE001
            print(f'  FALHOU  {nome:22s} {type(erro).__name__}: {erro}')
        else:
            passaram += 1
            print(f'  PASSOU  {nome:22s} {detalhe}')

    print(f'\n{passaram}/{len(CASOS)} passaram')
    return 0 if passaram == len(CASOS) else 1


if __name__ == '__main__':
    sys.exit(main())
