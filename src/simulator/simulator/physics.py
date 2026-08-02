"""Física do campo VSS. Sem ROS, sem I/O — só matemática, para poder testar.

O objetivo não é simular o robô com fidelidade de engenharia: é simular bem o
suficiente para escrever e afinar a IA antes do hardware existir. As coisas que
importam para isso são as que estão modeladas — inércia, atrito, colisão,
saturação e o fato de que roda não é velocidade.

Unidades: metros, segundos, radianos. Origem no centro do campo, x cresce para
a direita, y cresce para cima, ângulo 0 aponta para +x.
"""

import math
# Alias porque a dataclass World tem um atributo chamado `field`, que sombrearia
# a função field() dentro do corpo da classe.
from dataclasses import dataclass, field as dc_field


@dataclass
class FieldSpec:
    """Dimensões do campo. Os defaults são a IEEE VSS de 3x3."""

    length: float = 1.50        # eixo x
    width: float = 1.30         # eixo y
    goal_width: float = 0.40    # abertura do gol, no eixo y
    goal_depth: float = 0.10

    @property
    def half_length(self) -> float:
        return self.length / 2.0

    @property
    def half_width(self) -> float:
        return self.width / 2.0

    @property
    def half_goal(self) -> float:
        return self.goal_width / 2.0


@dataclass
class RobotSpec:
    radius: float = 0.0375       # metade dos 7,5 cm do cubo
    wheel_base: float = 0.0625   # distância entre as rodas
    wheel_speed_max: float = 0.75  # m/s de roda a PWM 100%
    mass: float = 0.20

    # Quanto o robô demora para atingir a velocidade comandada. Sem isto o robô
    # muda de direção instantaneamente e a IA fica boa demais — ela aprende a
    # contar com uma agilidade que o hardware não tem.
    accel_tau: float = 0.08


@dataclass
class BallSpec:
    radius: float = 0.02135      # bola de golfe, 42,7 mm
    friction: float = 1.2        # desaceleração, m/s²
    restitution: float = 0.55    # quique na parede
    max_speed: float = 3.0


@dataclass
class Robot:
    id: int
    x: float
    y: float
    theta: float
    vx: float = 0.0
    vy: float = 0.0
    vtheta: float = 0.0

    # Último comando recebido, em [-1, 1] por roda.
    cmd_left: float = 0.0
    cmd_right: float = 0.0

    # Velocidades efetivas do corpo, que perseguem o comando com atraso.
    v: float = 0.0
    omega: float = 0.0


@dataclass
class Ball:
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class World:
    field: FieldSpec = dc_field(default_factory=FieldSpec)
    robot_spec: RobotSpec = dc_field(default_factory=RobotSpec)
    ball_spec: BallSpec = dc_field(default_factory=BallSpec)
    robots: dict = dc_field(default_factory=dict)
    ball: Ball = dc_field(default_factory=Ball)

    # Preenchido pelo step() quando a bola entra no gol: 'left' ou 'right'.
    goal_event: str = None


def _clamp(value, low, high):
    return max(low, min(high, value))


def make_default_world(field_spec=None, robot_spec=None, ball_spec=None) -> World:
    """Campo com dois robôs em posição de kickoff: IA na esquerda, jogador na direita."""
    world = World(
        field=field_spec or FieldSpec(),
        robot_spec=robot_spec or RobotSpec(),
        ball_spec=ball_spec or BallSpec(),
    )
    reset_positions(world)
    return world


def reset_positions(world: World):
    """Recoloca tudo para o início de uma partida."""
    f = world.field

    world.robots[0] = Robot(id=0, x=-f.half_length * 0.55, y=0.0, theta=0.0)
    world.robots[1] = Robot(id=1, x=+f.half_length * 0.55, y=0.0, theta=math.pi)

    world.ball = Ball()
    world.goal_event = None


def set_wheel_command(world: World, robot_id: int, left: float, right: float):
    robot = world.robots.get(robot_id)
    if robot is None:
        return

    robot.cmd_left = _clamp(left, -1.0, 1.0)
    robot.cmd_right = _clamp(right, -1.0, 1.0)


def step(world: World, dt: float):
    """Avança a simulação em dt segundos."""
    world.goal_event = None

    for robot in world.robots.values():
        _integrate_robot(world, robot, dt)

    _integrate_ball(world, dt)

    for robot in world.robots.values():
        _collide_robot_ball(world, robot, dt)

    _separate_robots(world)

    _check_goal(world)


def _integrate_robot(world: World, robot: Robot, dt: float):
    spec = world.robot_spec

    # Cinemática direta diferencial. Note que partimos do comando de RODA: é
    # assim que o robô real recebe, e é o que mantém o simulador honesto quanto
    # ao que o sistema de fato controla.
    v_target = (robot.cmd_right + robot.cmd_left) / 2.0 * spec.wheel_speed_max
    omega_target = ((robot.cmd_right - robot.cmd_left)
                    * spec.wheel_speed_max / spec.wheel_base)

    # Resposta de primeira ordem: o motor não atinge a velocidade na hora.
    alpha = 1.0 - math.exp(-dt / spec.accel_tau) if spec.accel_tau > 0 else 1.0
    robot.v += (v_target - robot.v) * alpha
    robot.omega += (omega_target - robot.omega) * alpha

    robot.theta = _wrap_angle(robot.theta + robot.omega * dt)

    new_x = robot.x + robot.v * math.cos(robot.theta) * dt
    new_y = robot.y + robot.v * math.sin(robot.theta) * dt

    f = world.field
    limit_x = f.half_length - spec.radius
    limit_y = f.half_width - spec.radius

    # As paredes param o robô em vez de fazê-lo deslizar. Bater na parede e ficar
    # preso é um comportamento real que a IA precisa aprender a evitar.
    clamped_x = _clamp(new_x, -limit_x, limit_x)
    clamped_y = _clamp(new_y, -limit_y, limit_y)

    if clamped_x != new_x or clamped_y != new_y:
        robot.v *= 0.3

    robot.vx = (clamped_x - robot.x) / dt if dt > 0 else 0.0
    robot.vy = (clamped_y - robot.y) / dt if dt > 0 else 0.0
    robot.vtheta = robot.omega

    robot.x = clamped_x
    robot.y = clamped_y


def _integrate_ball(world: World, dt: float):
    ball = world.ball
    spec = world.ball_spec
    f = world.field

    speed = math.hypot(ball.vx, ball.vy)

    if speed > 1e-6:
        decel = min(speed, spec.friction * dt)
        scale = (speed - decel) / speed
        ball.vx *= scale
        ball.vy *= scale
    else:
        ball.vx = ball.vy = 0.0

    ball.x += ball.vx * dt
    ball.y += ball.vy * dt

    limit_x = f.half_length - spec.radius
    limit_y = f.half_width - spec.radius

    # Nas laterais a bola sempre quica.
    if abs(ball.y) > limit_y:
        ball.y = _clamp(ball.y, -limit_y, limit_y)
        ball.vy = -ball.vy * spec.restitution

    # No fundo ela só quica fora da boca do gol; dentro dela, segue e vira gol.
    if abs(ball.x) > limit_x and abs(ball.y) > f.half_goal:
        ball.x = _clamp(ball.x, -limit_x, limit_x)
        ball.vx = -ball.vx * spec.restitution


def _collide_robot_ball(world: World, robot: Robot, dt: float):
    ball = world.ball
    r_sum = world.robot_spec.radius + world.ball_spec.radius

    dx = ball.x - robot.x
    dy = ball.y - robot.y
    dist = math.hypot(dx, dy)

    if dist >= r_sum:
        return

    if dist < 1e-9:
        # Sobrepostos exatamente: empurra na direção que o robô aponta.
        nx, ny = math.cos(robot.theta), math.sin(robot.theta)
        dist = 1e-9
    else:
        nx, ny = dx / dist, dy / dist

    # Tira a sobreposição antes de aplicar impulso, senão a bola gruda no robô.
    overlap = r_sum - dist
    ball.x += nx * overlap
    ball.y += ny * overlap

    # Componente da velocidade do robô na direção da bola. Empurrar a bola só
    # funciona se o robô estiver de fato indo na direção dela.
    robot_speed_along_normal = robot.vx * nx + robot.vy * ny
    ball_speed_along_normal = ball.vx * nx + ball.vy * ny

    approach = robot_speed_along_normal - ball_speed_along_normal

    if approach > 0:
        # Transferência inelástica: a bola sai com um pouco mais que a velocidade
        # com que foi atingida. O 1.6 é empírico, calibrado para o chute parecer
        # com o que um robô VSS faz de verdade.
        impulse = approach * 1.6
        ball.vx += nx * impulse
        ball.vy += ny * impulse

    speed = math.hypot(ball.vx, ball.vy)
    if speed > world.ball_spec.max_speed:
        scale = world.ball_spec.max_speed / speed
        ball.vx *= scale
        ball.vy *= scale


def _separate_robots(world: World):
    """Impede que dois robôs ocupem o mesmo lugar."""
    ids = sorted(world.robots.keys())
    r = world.robot_spec.radius

    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a = world.robots[ids[i]]
            b = world.robots[ids[j]]

            dx = b.x - a.x
            dy = b.y - a.y
            dist = math.hypot(dx, dy)
            min_dist = 2 * r

            if dist >= min_dist or dist < 1e-9:
                continue

            overlap = (min_dist - dist) / 2.0
            nx, ny = dx / dist, dy / dist

            a.x -= nx * overlap
            a.y -= ny * overlap
            b.x += nx * overlap
            b.y += ny * overlap

            a.v *= 0.5
            b.v *= 0.5


def _check_goal(world: World):
    f = world.field
    ball = world.ball

    if abs(ball.y) > f.half_goal:
        return

    # Gol na esquerda significa ponto para quem ataca a esquerda: o jogador.
    if ball.x < -f.half_length + world.ball_spec.radius:
        world.goal_event = 'left'
    elif ball.x > f.half_length - world.ball_spec.radius:
        world.goal_event = 'right'


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
