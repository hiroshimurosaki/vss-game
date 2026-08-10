"""A topologia esperada do jogo, declarada — e o que cada elo significa.

Este arquivo é a razão de o painel existir. Um visualizador genérico de grafo
ROS mostra **o que existe**; para depurar, o que importa é **o que deveria
existir e não está**. Só dá para dizer isso quando a topologia certa está
escrita em algum lugar, e é aqui.

Espelha o desenho do `CLAUDE.md`. Se os dois divergirem, um dos dois está
errado — e é bom que quebre visível.

Cada elo carrega quatro coisas que um `ros2 topic hz` não dá:

    de/para       quem devia falar e quem devia ouvir, pelo nome real do nó
    resumo        uma linha em português do conteúdo, não o dump da mensagem
    porque_para   o que ACONTECE rio abaixo quando este elo morre
    normal_se     quando "parado" é o comportamento correto, não uma falha

O último é o mais importante. Metade das caças ao fantasma deste projeto foi
alguém tratando um silêncio projetado como defeito — a IA parada fora da
partida é o exemplo canônico, e está documentado como a primeira coisa a
conferir quando "a IA não faz nada".
"""

from dataclasses import dataclass, field
from typing import Callable, Optional

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty, String

from shared_interfaces.msg import (
    ActionsList, AiDebug, DirectionList, GameData, GameStatus,
    HighScoreList, JoyList, MotorVelocitiesList,
)


# ── Nomes reais dos nós ──────────────────────────────────────────────────────

NO_VISION   = 'vision_game'
NO_SIM      = 'simulator'
NO_MASTER   = 'game_master'
NO_IA       = 'ai_player'
NO_AGG      = 'joy_aggregator'
NO_DIRECAO  = 'direction'
NO_ESPECIAL = 'special_controls'
NO_CINEMA   = 'cinematica'
NO_RADIO    = 'radio_communication'
NO_TECLADO  = 'keyboard_input'

#: Nós que atendem por mais de um nome. A cinemática se registra no C++ como
#: `kinematics_node` (`Cinematica.cpp:6`) e o launch a renomeia para
#: `cinematica` (`game.py:217`) — então o nome que aparece no `ros2 node list`
#: depende de como ela foi subida. Procurar por um só dos dois faz o painel
#: anunciar que a cinemática morreu quando ela está viva com o outro nome, que
#: é exatamente o falso alarme que o painel existe para não dar.
ALIASES = {
    NO_CINEMA: (NO_CINEMA, 'kinematics_node'),
}


def nomes_de(no: str) -> tuple:
    """Todos os nomes por que um nó pode atender."""
    return ALIASES.get(no, (no,))


def presente(no: str, conjunto) -> bool:
    """O nó está neste conjunto, por qualquer um dos seus nomes?"""
    return any(alias in conjunto for alias in nomes_de(no))


@dataclass
class Elo:
    """Um tópico do caminho principal, com o que ele significa."""

    topico: str
    tipo: type
    de: tuple                     # nós que podem publicar (qualquer um serve)
    para: tuple                   # nós que deveriam estar ouvindo
    resumo: Callable              # msg -> str, uma linha em português
    porque_para: str              # o que morre rio abaixo se este elo morrer

    #: Taxa mínima esperada, não taxa exata: `/game_data` sai a 30 Hz da câmera
    #: e a 60 do simulador, e os dois estão certos. `None` = dirigido a evento,
    #: e aí silêncio não é sintoma.
    hz: Optional[float] = None

    #: (msg, ctx) -> str | None. Devolve texto quando o conteúdo, embora
    #: chegando, trava o pipeline. É o caso do `ball_detected=false`: o tópico
    #: está a 30 Hz, tudo "verde", e mesmo assim nada anda.
    #:
    #: O `ctx` carrega o estado da partida, lido do `/game/status`. Sem ele o
    #: painel acusa "motores zerados" na tela de atração, onde zero é o
    #: comportamento correto — e um painel que alarma sem motivo é um painel
    #: que se aprende a ignorar. Metade do valor daqui está em CALAR na hora
    #: certa.
    alerta: Optional[Callable] = None

    #: Quando parado é o certo. Mostrado no lugar do alarme.
    normal_se: str = ''

    #: Se dois nós publicando aqui ao mesmo tempo é uma FALHA.
    #:
    #: Só `/game_data` tem isso. A câmera e o simulador publicam o mesmo tópico
    #: de propósito, mas um de cada vez: os dois juntos dão ~74 Hz numa câmera
    #: de 30 e posições alternando entre duas fontes — o sintoma clássico que
    #: não aponta para a causa.
    #:
    #: Em `/ai/difficulty` dois publicadores são NORMAIS: o árbitro publica e o
    #: simulador também, porque tem seletor de dificuldade na própria tela.
    #: Tratar isso como falha foi um alarme falso do painel na primeira
    #: execução — daí este campo existir em vez de uma regra global.
    fonte_unica: bool = False

    #: Basta UM dos nós de `para` estar ouvindo.
    #:
    #: Existe porque a visão e o simulador são intercambiáveis por projeto:
    #: exatamente um dos dois roda. Sem isto o painel riscaria eternamente o
    #: que não subiu — um alarme que nunca apaga e que treina quem usa a
    #: ignorar a tela. Vale para `/ai/debug` e `/motorVelocities`, onde o
    #: consumidor depende de estar no simulador ou no campo de verdade.
    para_qualquer: bool = False

    #: Etapa da cadeia, para ordenar o veredito da origem ao motor.
    etapa: int = 0


def _fmt(v, casas=2):
    return f'{v:+.{casas}f}'


def _resumo_game_data(m):
    if not m.ball_detected:
        return f'{len(m.robots)} robô(s), bola NÃO detectada'
    return (f'{len(m.robots)} robô(s) · bola '
            f'({_fmt(m.ball.x)}, {_fmt(m.ball.y)}) m')


@dataclass
class Contexto:
    """O que o painel sabe sobre a partida na hora de julgar um elo.

    Só o estado do jogo, por enquanto. É o suficiente para separar "parado
    porque acabou" de "parado e não devia".
    """

    estado_jogo: str = ''

    @property
    def em_jogo(self) -> bool:
        return self.estado_jogo == 'JOGANDO'


def _alerta_game_data(m, ctx):
    if not m.ball_detected:
        return ('sem bola: a IA para e o árbitro ignora o frame — '
                'por projeto, não por defeito')
    if len(m.robots) < 2:
        return (f'só {len(m.robots)} robô no quadro: a IA para quando não se '
                f'enxerga ou não enxerga o adversário')
    return None


def _resumo_status(m):
    return (f'{m.state} · {m.player_name or "sem jogador"} '
            f'{m.ai_score}×{m.player_score} · {m.elapsed:.1f}s')


def _resumo_joy(m):
    eixos = ', '.join(f'{a:+.2f}' for a in list(m.axes)[:4])
    botoes = sum(1 for b in m.buttons if b)
    return f'eixos [{eixos}] · {botoes} botão(ões) apertado(s)'


def _alerta_joy(m, ctx):
    if not ctx.em_jogo:
        return None
    if all(abs(a) < 1e-3 for a in m.axes) and not any(m.buttons):
        return ('partida em andamento e o controle manda só zero: ou ninguém '
                'está mexendo, ou o gamepad não está chegando neste tópico')
    return None


def _resumo_motores(m):
    if not m.velocities:
        return 'lista vazia'
    return ' · '.join(f'robô {v.id}: {v.left:+.2f}/{v.right:+.2f}'
                      for v in m.velocities)


def _alerta_motores(m, ctx):
    """Motor zerado só é sintoma DENTRO da partida.

    Fora dela é o projeto funcionando: o árbitro tira a coleira da IA apenas
    em JOGANDO, e sem o `/ai/enabled` a IA manda zero. Acusar isso na tela de
    atração — que é onde o estande passa a maior parte do dia — encheria o
    painel de laranja permanente.
    """
    if not ctx.em_jogo:
        return None
    if m.velocities and all(abs(v.left) < 1e-3 and abs(v.right) < 1e-3
                            for v in m.velocities):
        return ('partida em andamento e TODAS as rodas em zero: a decisão não '
                'está virando movimento. Suba a cadeia — /direction, /joy_list '
                'e /ai/enabled, nessa ordem.')
    return None


def _resumo_direction(m):
    if not m.directions:
        return 'lista vazia'
    return ' · '.join(f'robô {d.id}: v={d.linear_vel:+.2f} w={d.angular_vel:+.2f}'
                      for d in m.directions)


def _resumo_ai_debug(m):
    return (f'{m.state} · alvo ({_fmt(m.target_x)}, {_fmt(m.target_y)}) · '
            f'{m.difficulty} · atraso {m.reaction_delay*1000:.0f} ms')


def _resumo_bool(m):
    return 'true' if m.data else 'false'


def _resumo_string(m):
    return m.data or '(vazio)'


ELOS = [
    Elo(
        topico='/game_data', tipo=GameData, etapa=1, hz=30.0, fonte_unica=True,
        de=(NO_VISION, NO_SIM), para=(NO_MASTER, NO_IA),
        resumo=_resumo_game_data, alerta=_alerta_game_data,
        porque_para='sem isto NADA anda: a IA não decide e o árbitro não '
                    'cronometra. É a fronteira do hardware.',
    ),
    Elo(
        topico='/ai/enabled', tipo=Bool, etapa=2,
        de=(NO_MASTER,), para=(NO_IA,),
        resumo=_resumo_bool,
        porque_para='a IA fica na coleira e manda zero.',
        normal_se='o árbitro só publica quando o estado MUDA. Silêncio com '
                  'partida parada é o esperado — e "false" fora de JOGANDO '
                  'está CERTO. É a primeira coisa a conferir quando alguém '
                  'diz que a IA não faz nada.',
    ),
    Elo(
        topico='/ai/difficulty', tipo=String, etapa=2,
        de=(NO_MASTER, NO_SIM), para=(NO_IA, NO_MASTER),
        resumo=_resumo_string,
        porque_para='a IA fica na última dificuldade que ouviu.',
        normal_se='dirigido a evento, e com DOIS publicadores por projeto: o '
                  'árbitro e o simulador, que tem seletor de dificuldade na '
                  'própria tela. O árbitro ainda assina o próprio tópico, para '
                  'um `ros2 topic pub` externo mudar a dificuldade e a TV '
                  'acompanhar.',
    ),
    Elo(
        topico='/joy_0', tipo=Joy, etapa=3, hz=30.0,
        de=(NO_IA,), para=(NO_AGG,),
        resumo=_resumo_joy,
        porque_para='o robô da IA não recebe comando nenhum.',
        normal_se='a IA publica mesmo desligada, mas com tudo em zero.',
    ),
    Elo(
        topico='/joy_1', tipo=Joy, etapa=3, hz=30.0,
        de=(NO_TECLADO, NO_SIM, 'game_controller_node'), para=(NO_AGG,),
        resumo=_resumo_joy, alerta=_alerta_joy,
        porque_para='o visitante aperta o controle e o robô não anda.',
        normal_se='o nó do gamepad é o `joy` do ROS, de fora deste repositório. '
                  'Sem controle plugado, ninguém publica aqui.',
    ),
    Elo(
        topico='/joy_list', tipo=JoyList, etapa=4, hz=30.0,
        de=(NO_AGG,), para=(NO_DIRECAO, NO_ESPECIAL),
        resumo=lambda m: f'{len(m.joys)} controle(s) agregado(s)',
        porque_para='nem a direção nem os controles especiais recebem nada.',
    ),
    Elo(
        topico='/direction', tipo=DirectionList, etapa=5, hz=30.0,
        de=(NO_DIRECAO,), para=(NO_CINEMA,),
        resumo=_resumo_direction,
        porque_para='a cinemática não tem o que converter em roda.',
    ),
    Elo(
        topico='/actions', tipo=ActionsList, etapa=5, hz=30.0,
        de=(NO_ESPECIAL,), para=(NO_CINEMA,),
        resumo=lambda m: f'{len(m.actions)} ação(ões)',
        porque_para='giro, chute e drible param de responder.',
    ),
    Elo(
        topico='/motorVelocities', tipo=MotorVelocitiesList, etapa=6, hz=30.0,
        de=(NO_CINEMA,), para=(NO_RADIO, NO_SIM), para_qualquer=True,
        resumo=_resumo_motores, alerta=_alerta_motores,
        porque_para='é o último elo que o software enxerga. Daqui para a '
                    'frente é serial, rádio e motor — e nenhum deles publica '
                    'nada de volta.',
    ),
    Elo(
        # Consumido pela visão e pelo simulador — conferido no código, não
        # supondo. O árbitro NÃO assina isto; incluí-lo aqui fez o painel
        # acusar o `game_master` de surdo na primeira execução.
        topico='/ai/debug', tipo=AiDebug, etapa=7, hz=30.0,
        de=(NO_IA,), para=(NO_VISION, NO_SIM), para_qualquer=True,
        resumo=_resumo_ai_debug,
        porque_para='o telão perde o conteúdo pedagógico — o alvo da IA e '
                    'onde ela acha que a bola está. O jogo continua.',
    ),
    Elo(
        topico='/game/status', tipo=GameStatus, etapa=7, hz=30.0,
        de=(NO_MASTER,), para=(),
        resumo=_resumo_status,
        porque_para='a TV e o painel do operador congelam no último estado.',
        normal_se='"ninguém" aqui está certo: quem consome isto são as telas, '
                  'por WebSocket na porta 8090, e WebSocket não aparece no '
                  'grafo do ROS. As GUIs deste projeto não são nós.',
    ),
    Elo(
        topico='/game/highscores', tipo=HighScoreList, etapa=7,
        de=(NO_MASTER,), para=(),
        resumo=lambda m: f'{len(m.entries)} no ranking',
        porque_para='o ranking do telão para de atualizar.',
        normal_se='publicado quando o ranking muda, não continuamente.',
    ),
    Elo(
        topico='/sim/reset', tipo=Empty, etapa=7,
        de=(NO_MASTER,), para=(NO_SIM,),
        resumo=lambda m: 'pedido de recolocação',
        porque_para='os robôs não voltam ao lugar depois do gol.',
        normal_se='dirigido a evento: um pulso por gol.',
    ),
]


#: Nós que deveriam existir num jogo completo. `alternativas` porque a visão e
#: o simulador são intercambiáveis por projeto — exatamente um dos dois deve
#: estar de pé, e os DOIS de pé é uma falha própria (ver painel).
NOS_ESPERADOS = [
    ('fonte de /game_data', (NO_VISION, NO_SIM), 'exclusivo'),
    ('árbitro',             (NO_MASTER,),        'obrigatório'),
    ('IA',                  (NO_IA,),            'obrigatório'),
    ('agregador de joy',    (NO_AGG,),           'obrigatório'),
    ('direção',             (NO_DIRECAO,),       'obrigatório'),
    ('controles especiais', (NO_ESPECIAL,),      'obrigatório'),
    ('cinemática',          (NO_CINEMA,),        'obrigatório'),
    ('rádio',               (NO_RADIO,),         'opcional'),
    ('teclado',             (NO_TECLADO,),       'opcional'),
]
