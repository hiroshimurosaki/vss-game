"""O cérebro da IA. Sem ROS, sem I/O — decide e devolve.

Separado do nó por dois motivos: dá para testar milhares de situações em
segundos sem subir o ROS, e dá para explicar a lógica para o público sem
precisar falar de tópicos.

## Como ela joga

Um robô diferencial não pode andar de lado. Se ele for direto na bola, empurra
para onde estiver apontando — que quase nunca é o gol. Então a IA sempre resolve
duas coisas em ordem:

  1. **Onde eu preciso estar** para que empurrar a bola a mande para o gol?
     Esse é o *ponto de ataque*: atrás da bola, na reta que liga a bola ao gol.
  2. **Já estou lá?** Se sim, acelera na bola. Se não, vai para o ponto de
     ataque contornando a bola, para não empurrá-la para o lado errado no
     caminho.

## As dificuldades

Não são um número abstrato de "força". Cada uma é uma limitação que existe de
verdade em robótica, e que dá para explicar apontando para o campo:

  speed_frac      — o robô não usa toda a força que tem
  reaction_delay  — ele te vê no passado (aplicado no nó, não aqui)
  replan_period   — ele só *decide* algumas vezes por segundo
  home_x_max      — ele não sai da própria metade: é zagueiro, não atacante
  aim_noise       — a visão dele erra alguns centímetros
"""

import math
from dataclasses import dataclass


# ── Estados, para a TV mostrar ───────────────────────────────────────────

ATACAR = 'ATACAR'
POSICIONAR = 'POSICIONAR'
DEFENDER = 'DEFENDER'
PARADO = 'PARADO'


@dataclass
class Difficulty:
    """Um preset de dificuldade."""

    name: str = 'MEDIO'

    # Fração da velocidade máxima que a IA se permite usar.
    speed_frac: float = 0.65

    # Segundos de atraso na percepção. Aplicado pelo nó, que segura os
    # snapshots; fica aqui só para ir junto no debug.
    reaction_delay: float = 0.25

    # A IA só recalcula o alvo a cada tantos segundos. Entre uma decisão e
    # outra ela persegue o alvo antigo — que é o que a faz parecer "burra"
    # quando o jogador muda a bola de lugar rápido.
    replan_period: float = 0.40

    # Até onde ela vai *buscar* a bola. Passando disso, ela desiste e volta a
    # defender o próprio gol. Não é um limite de posição: se ela já pegou a
    # bola, leva até o fim. Negativo = nem chega ao meio de campo.
    home_x_max: float = 0.10

    # Ruído somado ao alvo escolhido, em metros.
    aim_noise: float = 0.03

    # Distância atrás da bola onde ela se posiciona para atacar.
    approach_offset: float = 0.11

    # Quão alinhado ela exige estar antes de partir para cima da bola.
    # Em radianos: menor = mais exigente = joga melhor.
    attack_tolerance: float = 0.55


PRESETS = {
    'FACIL': Difficulty(
        name='FACIL',
        speed_frac=0.45,
        reaction_delay=0.40,
        replan_period=0.60,
        home_x_max=-0.15,      # nem chega ao meio: puro zagueiro
        aim_noise=0.06,
        attack_tolerance=0.80,
    ),
    'MEDIO': Difficulty(
        name='MEDIO',
        speed_frac=0.65,
        reaction_delay=0.25,
        replan_period=0.40,
        home_x_max=0.10,
        aim_noise=0.03,
        attack_tolerance=0.55,
    ),
    'DIFICIL': Difficulty(
        name='DIFICIL',
        speed_frac=0.90,
        reaction_delay=0.10,
        replan_period=0.15,
        home_x_max=0.55,       # cruza o campo e ataca
        aim_noise=0.01,
        attack_tolerance=0.35,
    ),
}


@dataclass
class Geometry:
    half_length: float = 0.75
    half_width: float = 0.65
    goal_half: float = 0.20
    robot_radius: float = 0.0375

    # Limites do controlador, em m/s e rad/s. Precisam bater com os do
    # DirectionNode, senão a conversão para Joy sai com escala errada.
    max_linear: float = 0.6
    max_angular: float = 5.0


@dataclass
class Decision:
    linear: float = 0.0
    angular: float = 0.0
    state: str = PARADO
    target_x: float = 0.0
    target_y: float = 0.0


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value, low, high):
    return max(low, min(high, value))


def choose_target(ball, me, geo: Geometry, diff: Difficulty, prev_state=None):
    """Decide para onde ir e por quê. Devolve (x, y, estado).

    `ball` e `me` são objetos/tuplas com .x e .y; `me` também tem .theta.
    `prev_state` dá histerese: uma vez atacando, ela desiste menos fácil. Sem
    isso a IA fica oscilando entre posicionar e atacar em cima da bola.
    """
    # A IA defende a esquerda, então ataca a direita.
    goal_x, goal_y = geo.half_length, 0.0
    own_goal_x = -geo.half_length

    # Vetor unitário da bola para o gol adversário.
    gx, gy = goal_x - ball.x, goal_y - ball.y
    gnorm = math.hypot(gx, gy)

    if gnorm < 1e-6:
        gx, gy = 1.0, 0.0
    else:
        gx, gy = gx / gnorm, gy / gnorm

    to_ball_x, to_ball_y = ball.x - me.x, ball.y - me.y
    distance_to_ball = math.hypot(to_ball_x, to_ball_y)

    # A bola está longe demais, na metade do adversário? Volta a defender. Um
    # robô que persegue a bola até o fim do campo deixa o gol aberto — e o
    # visitante aprende isso em dez segundos.
    #
    # Mas só desiste se de fato não estiver com a bola. Sem essa condição, a IA
    # empurra a bola até a borda da própria zona, o limite dispara, ela larga
    # tudo e volta — e nunca conclui uma jogada sequer. home_x_max é o alcance
    # da BUSCA, não uma coleira.
    ball_is_far_ahead = ball.x > diff.home_x_max + diff.approach_offset

    if ball_is_far_ahead and distance_to_ball > 0.25:
        defend_x = own_goal_x + 0.18
        defend_y = _clamp(ball.y * 0.55, -geo.goal_half, geo.goal_half)
        return defend_x, defend_y, DEFENDER

    # Quanto estou "atrás" da bola no eixo do ataque, e quanto estou fora da
    # reta bola->gol.
    alignment = to_ball_x * gx + to_ball_y * gy
    lateral = abs(-to_ball_x * gy + to_ball_y * gx)

    # Aproximação contínua, sem máquina de estados.
    #
    # Uma versão anterior classificava em ATACAR/POSICIONAR com um limiar, e a
    # IA travava: parada em cima do ponto de aproximação, com 2 cm de erro
    # lateral, ela oscilava entre os dois estados sem nunca sair — o alvo caía
    # dentro da zona morta do controlador e ela deixava de se mover.
    #
    # Aqui o alvo desliza conforme o alinhamento. Bem alinhada, ele fica ALÉM
    # da bola (ela atravessa e empurra). Desalinhada, ele recua para trás da
    # bola (ela contorna). Não há limiar para oscilar, e o alvo nunca fica
    # colado no robô.
    misalign = min(1.0, lateral / max(diff.approach_offset, 1e-6))

    # Estando na frente da bola, contornar é obrigatório, não uma questão de
    # grau: empurrar dali seria mandar a bola para o próprio gol.
    if alignment <= 0:
        misalign = 1.0

    # misalign 0 -> mira 10 cm além da bola;  1 -> mira approach_offset atrás.
    offset = -0.10 + (diff.approach_offset + 0.10) * misalign

    target_x = ball.x - gx * offset
    target_y = ball.y - gy * offset

    # O estado é só rótulo, para a TV explicar o que ela está fazendo.
    state = ATACAR if misalign < 0.45 else POSICIONAR

    return target_x, target_y, state


def go_to_point(me, target_x, target_y, geo: Geometry, diff: Difficulty,
                brake=True):
    """Controlador diferencial de ir-até-o-ponto. Devolve (linear, angular).

    Anda de ré quando o alvo está atrás: girar 180 graus custa mais tempo do
    que simplesmente inverter as rodas, e o robô VSS é simétrico.

    `brake=False` desliga a desaceleração ao chegar perto. No ataque queremos
    justamente atravessar o alvo com velocidade, não parar em cima dele.
    """
    dx, dy = target_x - me.x, target_y - me.y
    distance = math.hypot(dx, dy)

    if distance < 0.02:
        return 0.0, 0.0

    heading_error = _wrap(math.atan2(dy, dx) - me.theta)

    reverse = abs(heading_error) > math.pi / 2

    if reverse:
        heading_error = _wrap(heading_error - math.pi)

    # Gira proporcional ao erro, saturando no limite do controlador.
    angular = _clamp(3.2 * heading_error, -geo.max_angular, geo.max_angular)

    # Só acelera na medida em que está apontando para o alvo. O cos faz o robô
    # praticamente parar para girar quando está muito torto, em vez de sair
    # descrevendo um arco largo.
    speed_gain = math.cos(heading_error)
    speed_gain = max(0.0, speed_gain)

    # Freia ao chegar perto, para não passar direto do ponto de aproximação.
    approach_gain = min(1.0, distance / 0.20) if brake else 1.0

    linear = geo.max_linear * speed_gain * approach_gain

    if reverse:
        linear = -linear

    return linear, angular


def decide(ball, me, geo: Geometry, diff: Difficulty,
           cached_target=None, noise=(0.0, 0.0), prev_state=None):
    """Uma volta completa de decisão.

    `cached_target` permite que o nó reaproveite o alvo entre replanejamentos —
    é isso que dá o efeito de "ela só pensa 2 vezes por segundo".
    """
    if ball is None or me is None:
        return Decision(state=PARADO)

    if cached_target is None:
        target_x, target_y, state = choose_target(ball, me, geo, diff, prev_state)
        target_x += noise[0]
        target_y += noise[1]
    else:
        target_x, target_y, state = cached_target

    # home_x_max limita até onde ela vai *buscar* a bola — isso já foi decidido
    # em choose_target, que a manda defender quando a bola passa do limite.
    # Depois que ela pegou a bola, precisa poder levá-la até o gol; clampar aqui
    # também faria a IA empurrar até a borda da própria zona e travar sem
    # conseguir concluir a jogada.
    target_x = _clamp(target_x,
                      -geo.half_length + 0.05,
                      geo.half_length - 0.02)
    target_y = _clamp(target_y,
                      -geo.half_width + geo.robot_radius,
                      geo.half_width - geo.robot_radius)

    # No ataque ela atravessa o alvo com velocidade; nos outros estados ela
    # chega e para.
    linear, angular = go_to_point(me, target_x, target_y, geo, diff,
                                  brake=(state != ATACAR))

    # A dificuldade entra aqui, no fim: ela limita o quanto a IA se permite
    # usar do que decidiu. O raciocínio é o mesmo; só a execução é contida.
    linear *= diff.speed_frac
    angular *= diff.speed_frac

    return Decision(
        linear=linear,
        angular=angular,
        state=state,
        target_x=target_x,
        target_y=target_y,
    )


def to_joy_axes(linear, angular, geo: Geometry):
    """Converte (linear, angular) nos eixos de um Joy, convenção "signed".

    É o inverso exato do que o DirectionNode faz. Parece um rodeio — a IA
    poderia publicar Direction direto — mas passar pelo mesmo caminho do
    jogador garante que a IA sofre as mesmas saturações, os mesmos limites e
    os mesmos bugs que o humano. Se o pipeline mudar, ela muda junto.

    Devolve a lista de 6 eixos.
    """
    axes = [0.0] * 6

    throttle = _clamp(linear / geo.max_linear, -1.0, 1.0) if geo.max_linear else 0.0
    steer = _clamp(angular / geo.max_angular, -1.0, 1.0) if geo.max_angular else 0.0

    axes[0] = steer

    # Convenção signed: solto = +1.0, fundo = -1.0.
    axes[4] = 1.0   # L2, a ré
    axes[5] = 1.0   # R2, a frente

    if throttle >= 0:
        axes[5] = 1.0 - 2.0 * throttle
    else:
        axes[4] = 1.0 - 2.0 * (-throttle)

    return axes
