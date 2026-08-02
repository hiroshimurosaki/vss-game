#!/usr/bin/env python3
"""Mede a IA em lote, sem ROS e sem simulador rodando.

Junta o cérebro da IA com a física do simulador direto em memória e joga
centenas de partidas em segundos. Serve para responder "esse ajuste melhorou
ou piorou?" com número, em vez de olhar trinta segundos de tela e achar.

    ./tools/ai_bench.py                 # todos os presets
    ./tools/ai_bench.py --difficulty DIFICIL --trials 60
    ./tools/ai_bench.py --verbose       # mostra cada partida
"""

import argparse
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'ai_player'))
sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'simulator'))

from ai_player import brain          # noqa: E402
from simulator import physics        # noqa: E402


class _P:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta


def play(diff, seed, timeout=25.0, dt=1 / 60.0, opponent_parked=True):
    """Uma partida com o gol livre. Devolve (fez_gol, segundos, motivo)."""
    rng = random.Random(seed)

    world = physics.make_default_world()
    geo = brain.Geometry(
        half_length=world.field.half_length,
        half_width=world.field.half_width,
        goal_half=world.field.half_goal,
    )

    # Bola e IA em posições variadas, para não medir um único cenário de sorte.
    world.ball.x = rng.uniform(-0.35, 0.35)
    world.ball.y = rng.uniform(-0.40, 0.40)

    world.robots[0].x = rng.uniform(-0.65, -0.10)
    world.robots[0].y = rng.uniform(-0.45, 0.45)
    world.robots[0].theta = rng.uniform(-math.pi, math.pi)

    if opponent_parked:
        # Adversário fora do caminho: mede só a habilidade de concluir.
        world.robots[1].x = 0.62
        world.robots[1].y = 0.56

    cached = None
    last_state = None
    last_plan = -999.0
    t = 0.0

    while t < timeout:
        me = _P(world.robots[0].x, world.robots[0].y, world.robots[0].theta)
        ball = _P(world.ball.x, world.ball.y)

        if t - last_plan >= diff.replan_period:
            last_plan = t
            cached = None

        noise = (0.0, 0.0)
        if cached is None and diff.aim_noise > 0:
            noise = (rng.gauss(0, diff.aim_noise), rng.gauss(0, diff.aim_noise))

        decision = brain.decide(ball, me, geo, diff,
                                cached_target=cached, noise=noise,
                                prev_state=last_state)

        cached = (decision.target_x, decision.target_y, decision.state)
        last_state = decision.state

        # Passa pelo mesmo caminho do jogador: Joy -> DirectionNode -> cinemática.
        axes = brain.to_joy_axes(decision.linear, decision.angular, geo)
        linear, angular = _direction_node(axes, geo)
        left, right = _kinematics(linear, angular)

        physics.set_wheel_command(world, 0, left, right)
        physics.step(world, dt)

        if world.goal_event == 'right':
            return True, t, 'gol'
        if world.goal_event == 'left':
            return False, t, 'gol contra'

        t += dt

    return False, timeout, 'tempo'


def _direction_node(axes, geo):
    """Réplica exata do DirectionNode, para o benchmark medir o caminho real."""
    def norm(raw):
        return max(0.0, min(1.0, (1.0 - raw) / 2.0))

    throttle = max(-1.0, min(1.0, norm(axes[5]) - norm(axes[4])))
    return throttle * geo.max_linear, axes[0] * geo.max_angular


def _kinematics(linear, angular, axle=0.0625, wheel_max=0.75):
    """Réplica da Cinematica, incluindo a normalização proporcional."""
    right = (linear + angular * axle / 2.0) / wheel_max
    left = (linear - angular * axle / 2.0) / wheel_max

    peak = max(abs(left), abs(right))
    if peak > 1.0:
        left /= peak
        right /= peak

    return left, right


def bench(name, diff, trials, verbose=False):
    goals = 0
    times = []
    own_goals = 0

    for seed in range(trials):
        scored, elapsed, reason = play(diff, seed)

        if scored:
            goals += 1
            times.append(elapsed)
        elif reason == 'gol contra':
            own_goals += 1

        if verbose:
            print(f'    seed {seed:3d}  {reason:10s} {elapsed:5.1f}s')

    rate = goals / trials
    median = sorted(times)[len(times) // 2] if times else float('nan')

    print(f'  {name:8s}  gol em {rate:5.1%} das tentativas  '
          f'| mediana {median:5.1f}s  | gol contra {own_goals}')

    return rate, median


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--trials', type=int, default=40)
    parser.add_argument('--difficulty', default=None)
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()

    names = [args.difficulty.upper()] if args.difficulty else list(brain.PRESETS)

    print(f'IA contra gol livre, {args.trials} partidas de até 25 s cada\n')

    for name in names:
        if name not in brain.PRESETS:
            print(f'  dificuldade desconhecida: {name}')
            continue
        bench(name, brain.PRESETS[name], args.trials, args.verbose)

    print('\n  Referência: DIFICIL deveria concluir quase sempre e rápido;')
    print('  FACIL raramente ataca, então taxa baixa aqui é o esperado.')


if __name__ == '__main__':
    main()
