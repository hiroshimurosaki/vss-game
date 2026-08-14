#!/usr/bin/env python3
"""Roda partidas de duelo inteiras com relógio falso, sem ROS.

    ./tools/duelo_bench.py

O `duelo.py` é puro de propósito — o tempo entra por parâmetro — e é isso que
permite conferir uma tarde de feira em milissegundos. Este arquivo é o teste de
regressão do modo duelo, na mesma função que o `ai_bench.py` cumpre para a IA:
se alguém mexer nas regras e quebrar uma invariante, é aqui que aparece antes
de aparecer na frente de uma criança.

Cada caso confere uma coisa que custaria caro descobrir na feira:

    vitória/derrota      o placar de rounds e o tempo que vai para o ranking
    empate em série      que a partida TERMINA mesmo quando ninguém marca
    preparo              que ele espera o campo, mas nunca para sempre
    gol contra           que não conta e não encerra o turno de graça
    volante              que nunca há dois motoristas ao mesmo tempo
    pausa                que o tempo parado não conta contra o visitante
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'game_master'))

from game_master import duelo                     # noqa: E402
from game_master.rules import IA, JOGADOR         # noqa: E402

HALF_LEN = 0.75
GOAL_HALF = 0.20


class Sim:
    def __init__(self, **cfg):
        self.e = duelo.Engine(config=duelo.Config(**cfg))
        self.t = 0.0

    def avanca(self, dt, ready=True, passo=0.05):
        """Anda dt segundos, com o campo pronto (ou não), sem gol."""
        alvo = self.t + dt
        while self.t < alvo - 1e-9:
            self.t = min(self.t + passo, alvo)
            self.e.set_ready(ready, ready)
            self.e.on_ball(self.t, 0.0, 0.0, HALF_LEN, GOAL_HALF)
            self.e.tick(self.t)
        return self.e.match

    def gol(self):
        """Bola no gol da direita, agora."""
        r = self.e.on_ball(self.t, HALF_LEN + 0.01, 0.0, HALF_LEN, GOAL_HALF)
        self.e.tick(self.t)
        return r

    def ate(self, estado, teto=120.0):
        """Avança até entrar no estado pedido."""
        gasto = 0.0
        while self.e.match.state != estado and gasto < teto:
            self.avanca(0.05)
            gasto += 0.05
        assert self.e.match.state == estado, \
            f'esperava {estado}, ficou em {self.e.match.state}'
        return self.e.match


def turno(sim, segundos=None):
    """Joga um turno inteiro: espera o TURNO, gasta o tempo, faz (ou não) gol."""
    sim.ate(duelo.TURNO)
    quem = sim.e.match.driver

    if segundos is None:                       # estoura o teto sem marcar
        sim.avanca(sim.e.config.turn_limit + 0.2)
        return quem, None

    sim.avanca(segundos)
    marcador = sim.gol()
    assert marcador == quem, f'gol creditado a {marcador}, não a {quem}'
    return quem, segundos


def caso_vitoria_do_jogador():
    sim = Sim()
    sim.e.start(0.0, 'FERNANDO')
    assert sim.e.match.state == duelo.PREPARO
    assert sim.e.match.driver == JOGADOR

    # Round 1: jogador 8 s, Franky 12 s -> ponto do jogador.
    q, _ = turno(sim, 8.0);  assert q == JOGADOR
    q, _ = turno(sim, 12.0); assert q == IA
    sim.ate(duelo.ROUND)
    r1 = sim.e.match.rounds[-1]
    assert r1.winner == JOGADOR, r1
    assert (sim.e.match.player_rounds, sim.e.match.ai_rounds) == (1, 0)

    # Round 2: jogador 15 s, Franky 9 s -> ponto do Franky.
    q, _ = turno(sim, 15.0); assert q == JOGADOR
    q, _ = turno(sim, 9.0);  assert q == IA
    sim.ate(duelo.ROUND)
    assert (sim.e.match.player_rounds, sim.e.match.ai_rounds) == (1, 1)

    # Round 3: jogador 7 s, Franky estoura o teto -> jogador leva a partida.
    q, _ = turno(sim, 7.0);  assert q == JOGADOR
    q, _ = turno(sim, None); assert q == IA
    sim.ate(duelo.FIM)

    m = sim.e.match
    assert m.player_rounds == 2 and m.ai_rounds == 1, (m.player_rounds, m.ai_rounds)
    assert m.player_won is True
    esperado = 8.0 + 15.0 + 7.0
    assert abs(m.final_time - esperado) < 0.3, (m.final_time, esperado)
    assert len(m.rounds) == 3
    print(f'  vitória do jogador: 2x1 rounds, tempo somado {m.final_time:.2f} s '
          f'(esperado {esperado:.2f})')


def caso_derrota():
    sim = Sim()
    sim.e.start(0.0, 'AZARADO')
    for _ in range(2):
        turno(sim, 20.0)   # jogador lento
        turno(sim, 5.0)    # Franky rápido
        sim.ate(duelo.ROUND)
    sim.ate(duelo.FIM)
    m = sim.e.match
    assert (m.player_rounds, m.ai_rounds) == (0, 2), (m.player_rounds, m.ai_rounds)
    assert m.player_won is False
    print(f'  derrota: 0x2 rounds, não entra no ranking (player_won={m.player_won})')


def caso_empate_em_serie():
    """Ninguém marca nunca. Tem que terminar no max_rounds, não rodar para sempre."""
    sim = Sim(max_rounds=3)
    sim.e.start(0.0, 'NINGUEM')
    for _ in range(3):
        turno(sim, None)
        turno(sim, None)
        sim.ate(duelo.ROUND)
    sim.ate(duelo.FIM)
    m = sim.e.match
    assert m.round_number == 3
    assert (m.player_rounds, m.ai_rounds) == (0, 0)
    assert m.player_won is False
    assert all(r.winner == duelo.EMPATE for r in m.rounds)
    print(f'  empate em série: parou no round {m.round_number}, sem vencedor')


def caso_preparo_espera_o_campo():
    sim = Sim(prep_min=3.0, prep_max=20.0)
    sim.e.start(0.0, 'X')

    # Campo NÃO pronto: passa muito do piso e continua no preparo.
    sim.avanca(10.0, ready=False)
    assert sim.e.match.state == duelo.PREPARO, sim.e.match.state

    # Campo pronto: solta a contagem.
    sim.avanca(0.2, ready=True)
    assert sim.e.match.state == duelo.CONTAGEM, sim.e.match.state
    print('  preparo segura enquanto o campo não está pronto e solta quando fica')


def caso_preparo_tem_teto():
    sim = Sim(prep_min=3.0, prep_max=8.0)
    sim.e.start(0.0, 'X')
    sim.avanca(9.0, ready=False)   # nunca fica pronto
    assert sim.e.match.state in (duelo.CONTAGEM, duelo.TURNO), sim.e.match.state
    print('  preparo estoura o teto e começa assim mesmo: a fila não trava')


def caso_gol_contra_nao_conta():
    sim = Sim()
    sim.e.start(0.0, 'X')
    sim.ate(duelo.TURNO)
    marcador = sim.e.on_ball(sim.t, -HALF_LEN - 0.01, 0.0, HALF_LEN, GOAL_HALF)
    assert marcador is None
    assert sim.e.match.state == duelo.TURNO
    print('  gol no lado errado não conta e não encerra o turno')


def caso_volante():
    sim = Sim()
    sim.e.start(0.0, 'X')
    assert sim.e.joy_source() == IA and sim.e.ai_should_go_home()

    sim.ate(duelo.CONTAGEM)
    assert sim.e.joy_source() == '', 'ninguém dirige durante a contagem'
    assert not sim.e.ai_should_go_home()

    sim.ate(duelo.TURNO)
    assert sim.e.joy_source() == JOGADOR
    sim.avanca(5.0); sim.gol()
    assert sim.e.joy_source() == '', 'ninguém dirige durante a comemoração'

    sim.ate(duelo.PREPARO)
    assert sim.e.joy_source() == IA and sim.e.ai_should_go_home()
    sim.ate(duelo.TURNO)
    assert sim.e.joy_source() == IA and not sim.e.ai_should_go_home()
    print('  volante: IA no preparo, ninguém na contagem, o motorista no turno')


def caso_pausa():
    sim = Sim()
    sim.e.start(0.0, 'X')
    sim.ate(duelo.TURNO)
    sim.avanca(5.0)
    antes = sim.e.match.elapsed

    sim.e.toggle_pause(sim.t)
    sim.avanca(30.0)                    # meia eternidade parado
    assert sim.e.match.state == duelo.PAUSA
    sim.e.toggle_pause(sim.t)

    sim.avanca(1.0)
    depois = sim.e.match.elapsed
    assert abs(depois - (antes + 1.0)) < 0.3, (antes, depois)
    print(f'  pausa: {antes:.1f} s -> {depois:.1f} s, os 30 s parados não contaram')


# ── O tempo do Franky, medido na posição exata do duelo ──────────────────
#
# O número mais importante da feira inteira: é a marca que o visitante tem que
# bater. Se o Franky faz em 8 s, ninguém ganha e a fila esvazia; se faz em 25 s,
# o teto do turno precisa ser maior que isso ou ele mesmo não conclui.
#
# Mede o que a partida mede: robô na marca, bola no centro, gol da direita.
# Sem aleatoriedade de posição — a variação que sobra é a do próprio preset
# (ruído de mira e período de replanejamento), que é a que existe no campo.

def tempo_do_franky(nome_preset, tentativas=40, teto=45.0, dt=1 / 60.0,
                    home_x_max=None, jogo_ou_duelo='duelo'):
    import random
    sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'ai_player'))
    sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'simulator'))

    from ai_player import brain
    from simulator import physics
    import ai_bench

    conjunto = brain.CONJUNTOS[jogo_ou_duelo]
    preset = brain.Difficulty(**vars(conjunto[nome_preset]))

    if home_x_max is not None:
        preset.home_x_max = home_x_max
    tempos = []
    falhas = 0

    for semente in range(tentativas):
        rng = random.Random(semente)
        world = physics.make_default_world(robot_count=1)
        geo = brain.Geometry(
            half_length=world.field.half_length,
            half_width=world.field.half_width,
            goal_half=world.field.half_goal,
        )

        cached, last_state, last_plan, t = None, None, -999.0, 0.0

        while t < teto:
            me = ai_bench._P(world.robots[0].x, world.robots[0].y,
                             world.robots[0].theta)
            ball = ai_bench._P(world.ball.x, world.ball.y)

            if t - last_plan >= preset.replan_period:
                last_plan = t
                cached = None

            ruido = (0.0, 0.0)
            if cached is None and preset.aim_noise > 0:
                ruido = (rng.gauss(0, preset.aim_noise),
                         rng.gauss(0, preset.aim_noise))

            d = brain.decide(ball, me, geo, preset, cached_target=cached,
                             noise=ruido, prev_state=last_state)
            cached = (d.target_x, d.target_y, d.state)
            last_state = d.state

            axes = brain.to_joy_axes(d.linear, d.angular, geo)
            linear, angular = ai_bench._direction_node(axes, geo)
            left, right = ai_bench._kinematics(linear, angular)

            physics.set_wheel_command(world, 0, left, right)
            physics.step(world, dt)

            if world.goal_event == 'right':
                tempos.append(t)
                break
            if world.goal_event == 'left':
                falhas += 1
                break

            t += dt
        else:
            falhas += 1

    return tempos, falhas


def relatorio_do_franky(tentativas=40, jogo_ou_duelo='duelo'):
    import statistics

    print(f'\ntempo do Franky do começo do turno até o gol — presets '
          f'"{jogo_ou_duelo}" ({tentativas} tentativas por preset):\n')
    print(f'  {"preset":9s} {"conclui":>8s} {"mediana":>9s} {"p10":>7s} '
          f'{"p90":>7s} {"pior":>7s}')

    for nome in ('FACIL', 'MEDIO', 'DIFICIL'):
        tempos, falhas = tempo_do_franky(nome, tentativas,
                                         jogo_ou_duelo=jogo_ou_duelo)

        if not tempos:
            print(f'  {nome:9s} {"0%":>8s}   nunca concluiu')
            continue

        tempos.sort()
        conclui = len(tempos) / (len(tempos) + falhas)
        p10 = tempos[int(len(tempos) * 0.10)]
        p90 = tempos[min(len(tempos) - 1, int(len(tempos) * 0.90))]

        print(f'  {nome:9s} {conclui:7.0%} {statistics.median(tempos):8.1f}s '
              f'{p10:6.1f}s {p90:6.1f}s {tempos[-1]:6.1f}s')

    print('\nO teto do turno (turn_limit) precisa ser maior que o "pior" do '
          '\npreset em uso, senão o próprio Franky estoura o tempo.')


if __name__ == '__main__':
    if '--franky' in sys.argv:
        relatorio_do_franky()
        if '--comparar' in sys.argv:
            # Os presets do jogo, no cenário do duelo. Existe para não deixar
            # ninguém "simplificar" o PRESETS_DUELO de volta para o PRESETS.
            relatorio_do_franky(jogo_ou_duelo='jogo')
        sys.exit(0)

    for caso in (caso_vitoria_do_jogador, caso_derrota, caso_empate_em_serie,
                 caso_preparo_espera_o_campo, caso_preparo_tem_teto,
                 caso_gol_contra_nao_conta, caso_volante, caso_pausa):
        print(f'{caso.__name__}:')
        caso()
    print('\ntudo passou.')
    relatorio_do_franky(tentativas=20)
