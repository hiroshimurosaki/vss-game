"""Painel de fluxo: quem envia o que para quem, e onde a informação morre.

    ros2 run diagnostics flow_panel
    # abre http://localhost:8050

O `radio_console.py` responde "o robô anda?". O `debug_panel.py` responde
"onde a informação morre na bancada?", da serial ao PWM. Este responde a mesma
pergunta do lado do ROS, com o jogo rodando: **da câmera até o último tópico
que o software enxerga.**

Existe porque quase toda falha desta cadeia tem o mesmo sintoma — robô parado,
nenhum erro em lugar nenhum — e por isso o sintoma não aponta para a causa.

POR QUE NÃO É UM `rqt_graph`
    Um visualizador de grafo mostra o que existe. Para depurar, o que importa é
    o que DEVERIA existir e não está: o tópico que ninguém publica, o nó que
    morreu, o assinante que sumiu. Isso exige a topologia certa escrita em
    algum lugar — está em `topologia.py`, espelhando o `CLAUDE.md`.

POR QUE É UM NÓ SEPARADO E NÃO PARTE DO `game_master`
    Porque uma das falhas que ele precisa reportar é o `game_master` estar
    morto. Diagnóstico que morre junto com o paciente não serve.

O QUE ELE NÃO VÊ
    O rádio. Com o jogo rodando, quem tem a serial da ponte é o
    `radio_communication`, e serial não se abre duas vezes. Do
    `/motorVelocities` para a frente este painel diz apenas "entreguei" — se o
    robô não anda mesmo assim, é caso para `./tools/debug_panel.py`, na
    bancada, com o stack parado. O painel diz isso na tela em vez de deixar
    você procurando.
"""

import asyncio
import os
import threading
import time
from collections import deque

import rclpy
from aiohttp import web
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from .topologia import ELOS, NOS_ESPERADOS, Contexto, nomes_de, presente

#: Quantos segundos sem mensagem antes de um elo periódico virar "morto". Três
#: períodos de 30 Hz seriam 100 ms — apertado demais, qualquer engasgo de
#: escalonamento acenderia alarme. Um segundo é folgado e ainda assim imediato
#: na escala de quem está depurando de pé.
SILENCIO_S = 1.0

#: Abaixo desta fração da taxa nominal o elo está "arrastando". 0,6 porque a
#: visão a 30 Hz cai para ~20 quando a luz piora, e isso é digno de aviso sem
#: ser falha.
FRACAO_LENTA = 0.6


class Taxa:
    """Frequência por janela deslizante.

    Janela de tempo e não de contagem: um tópico que parou precisa cair para
    zero sozinho, e uma janela de N amostras fica congelada na última taxa
    para sempre — mostrando 30 Hz de um tópico que morreu há dez minutos.
    """

    def __init__(self, janela=2.0):
        self.janela = janela
        self._t = deque()

    def marcar(self, agora):
        self._t.append(agora)
        self._podar(agora)

    def hz(self, agora):
        self._podar(agora)
        if len(self._t) < 2:
            return 0.0
        span = self._t[-1] - self._t[0]
        return (len(self._t) - 1) / span if span > 0 else 0.0

    def _podar(self, agora):
        limite = agora - self.janela
        while self._t and self._t[0] < limite:
            self._t.popleft()


class EstadoElo:
    def __init__(self, elo):
        self.elo = elo
        self.taxa = Taxa()
        self.ultimo_t = None      # monotônico do último recebimento
        self.total = 0
        self.resumo = ''
        self.alerta = None


class FlowPanel(Node):

    def __init__(self):
        super().__init__('flow_panel')

        self.declare_parameter('port', 8050)
        self.port = int(self.get_parameter('port').value)

        self._lock = threading.Lock()
        self._elos = {e.topico: EstadoElo(e) for e in ELOS}
        self._t0 = time.monotonic()

        # O painel se informa pelo próprio `/game/status` para saber se o
        # silêncio que está vendo é o projetado. Ver `Contexto`.
        self._ctx = Contexto()

        for elo in ELOS:
            self.create_subscription(
                elo.tipo, elo.topico, self._fazer_callback(elo), 10)

        self._start_web_server()
        self.get_logger().info(
            f'Painel de fluxo em http://localhost:{self.port}')

    # ── ROS ────────────────────────────────────────────────────────────────

    def _fazer_callback(self, elo):
        """Fecha sobre o elo para não precisar de um método por tópico.

        O resumo é calculado AQUI e não na hora de servir a página: guardar a
        mensagem inteira e formatar depois seguraria uma referência viva a
        cada mensagem de cada tópico, e a serialização de `GameData` a 30 Hz
        não é de graça. Uma string por tópico é o que sobra.
        """
        estado = self._elos[elo.topico]

        def callback(msg):
            agora = time.monotonic()
            with self._lock:
                estado.taxa.marcar(agora)
                estado.ultimo_t = agora
                estado.total += 1

                # O estado da partida entra no contexto ANTES dos alertas, para
                # que o julgamento deste ciclo já use a informação mais nova.
                if elo.topico == '/game/status':
                    self._ctx.estado_jogo = msg.state

                try:
                    estado.resumo = elo.resumo(msg)
                    estado.alerta = elo.alerta(msg, self._ctx) if elo.alerta else None
                except Exception as exc:            # noqa: BLE001
                    # Resumo é conveniência: se um campo mudou de nome, o
                    # painel mostra o defeito em vez de morrer calado e fazer
                    # parecer que o tópico parou.
                    estado.resumo = f'(falha ao resumir: {exc})'
                    estado.alerta = None

        return callback

    # ── Diagnóstico ────────────────────────────────────────────────────────

    def _vivos(self):
        """Nós vivos agora, sem o namespace."""
        try:
            return {n for n, _ in self.get_node_names_and_namespaces()}
        except Exception:                            # noqa: BLE001
            return set()

    def _publicadores(self, topico):
        try:
            return [p.node_name for p in self.get_publishers_info_by_topic(topico)]
        except Exception:                            # noqa: BLE001
            return []

    def _assinantes(self, topico):
        """Assinantes reais — o próprio painel não conta.

        Ele assina TODOS os tópicos da topologia para poder medi-los, então
        apareceria como ouvinte de tudo. Mostrar isso faria a tela responder
        "quem ouve? o painel", que é verdade e é inútil — e pior, encheria a
        linha justamente nos casos em que ninguém mais está ouvindo, que é a
        falha que a linha existe para tornar visível.
        """
        try:
            return [s.node_name
                    for s in self.get_subscriptions_info_by_topic(topico)
                    if s.node_name != self.get_name()]
        except Exception:                            # noqa: BLE001
            return []

    def _estado_elo(self, est, agora, vivos):
        """Classifica um elo em ok / warn / bad / unknown, com explicação.

        A ordem das perguntas é a ordem em que elas são úteis para quem está
        depurando: primeiro "alguém devia estar falando?", só depois "está
        chegando na taxa certa?". Diagnosticar taxa de um tópico que ninguém
        publica é ruído.
        """
        elo = est.elo
        pubs = self._publicadores(elo.topico)
        subs = self._assinantes(elo.topico)
        hz = est.taxa.hz(agora)
        idade = None if est.ultimo_t is None else agora - est.ultimo_t

        detalhe = est.resumo or '—'
        dica = ''

        # 1. Ninguém publica: o elo não existe, não é lento.
        if not pubs:
            candidatos = ' ou '.join(elo.de)
            vivo_algum = any(presente(n, vivos) for n in elo.de)
            if vivo_algum:
                dica = (f'{candidatos} está de pé mas não abriu publicador '
                        f'neste tópico. Nome de tópico ou remapeamento.')
            else:
                dica = f'{candidatos} não está rodando.'
            return ('bad', 'ninguém publica', dica, hz, idade, pubs, subs)

        # 2. Duas fontes onde só cabe uma. Detectar isto é metade do motivo do
        #    painel: um simulador órfão junto da câmera dá ~74 Hz numa câmera
        #    de 30 e posições alternando entre duas fontes — sintoma que não
        #    aponta para a causa.
        #
        #    Por elo e não como regra global: em `/ai/difficulty` dois
        #    publicadores são normais. A primeira versão disto acusava lá, o
        #    que teria treinado quem usa o painel a ignorar o alarme.
        if elo.fonte_unica and len(set(pubs)) > 1:
            return ('bad', f'DUAS fontes publicando: {", ".join(sorted(set(pubs)))}',
                    'Um dos dois está órfão. As posições vão alternar entre as '
                    'duas fontes e a taxa vai somar. Derrube tudo com '
                    './tools/stop.sh e suba de novo.',
                    hz, idade, pubs, subs)

        # 3. Assinante SURDO: está de pé e mesmo assim não está ouvindo. É
        #    diferente de ausente, e a diferença é a única que importa aqui —
        #    nó que nem subiu já é reportado pela lista de nós, e repetir isso
        #    em toda linha viraria ruído. Surdo é sempre defeito: nome de
        #    tópico errado, remapeamento, ou assinatura que não foi criada.
        faltando = [n for n in elo.para
                    if presente(n, vivos) and not presente(n, subs)]

        #    Com `para_qualquer`, basta um ouvinte: visão e simulador não
        #    coexistem, então cobrar os dois seria um alarme que nunca apaga.
        if elo.para_qualquer and any(presente(n, subs) for n in elo.para):
            faltando = []
        if faltando:
            dica = (f'{", ".join(faltando)} está de pé mas não assina isto. '
                    f'{elo.porque_para}')

        # 4. Nunca chegou nada.
        if est.ultimo_t is None:
            desde = agora - self._t0
            if elo.hz is None:
                return ('unknown', 'nada desde que o painel abriu',
                        elo.normal_se or 'dirigido a evento.',
                        hz, idade, pubs, subs)
            return ('bad', f'publicador existe, mas nada chegou em {desde:.0f}s',
                    dica or elo.porque_para, hz, idade, pubs, subs)

        # 5. Dirigido a evento: idade é informação, não alarme.
        if elo.hz is None:
            return ('ok', f'{detalhe} · último há {idade:.1f}s',
                    dica or elo.normal_se, hz, idade, pubs, subs)

        # 6. Periódico: silêncio, arrasto, ou saudável.
        if idade > SILENCIO_S:
            return ('bad', f'parou há {idade:.1f}s (último: {detalhe})',
                    dica or elo.porque_para, hz, idade, pubs, subs)

        if hz < elo.hz * FRACAO_LENTA:
            return ('warn', f'{detalhe} — arrastando',
                    dica or f'esperado ~{elo.hz:.0f} Hz.',
                    hz, idade, pubs, subs)

        # 7. Chegando na taxa certa, mas com conteúdo que trava o pipeline.
        #    É o caso mais traiçoeiro: tudo verde e nada anda.
        if est.alerta:
            return ('warn', detalhe, est.alerta, hz, idade, pubs, subs)

        return ('ok', detalhe, dica, hz, idade, pubs, subs)

    @staticmethod
    def _rota(esperados, reais, vivos, verbo):
        """Um lado da rota, com cada nó já julgado.

        O julgamento fica aqui e não no JavaScript porque só aqui existe a
        informação dos três estados possíveis, e confundi-los é o que faz um
        painel mentir:

            ativo    está de pé e está falando/ouvindo — o normal
            surdo    está de pé e NÃO está: defeito, e o único acionável
            ausente  nem subiu: informação, não alarme
            extra    está falando/ouvindo sem ser esperado: ou a topologia
                     está desatualizada, ou subiu algo que não devia

        A primeira versão disto deixava o JS riscar todo esperado que não
        aparecesse, sem distinguir surdo de ausente. No modo simulador isso
        riscava `radio_communication` para sempre, num vermelho que não
        significava nada.
        """
        saida = []
        for nome in esperados:
            if presente(nome, reais):
                estado = 'ativo'
            elif presente(nome, vivos):
                estado = 'surdo'
            else:
                estado = 'ausente'
            saida.append({'nome': nome, 'estado': estado, 'verbo': verbo})

        conhecidos = {alias for n in esperados for alias in nomes_de(n)}
        for nome in reais:
            if nome not in conhecidos:
                saida.append({'nome': nome, 'estado': 'extra', 'verbo': verbo})

        return saida

    def _montar(self):
        agora = time.monotonic()
        vivos = self._vivos()

        with self._lock:
            elos = []
            for elo in ELOS:
                est = self._elos[elo.topico]
                status, detalhe, dica, hz, idade, pubs, subs = \
                    self._estado_elo(est, agora, vivos)
                elos.append({
                    'topico': elo.topico,
                    'tipo': elo.tipo.__name__,
                    'etapa': elo.etapa,
                    'rota_de': self._rota(elo.de, pubs, vivos, 'publicando'),
                    'rota_para': self._rota(elo.para, subs, vivos, 'ouvindo'),
                    'status': status,
                    'detalhe': detalhe,
                    'dica': dica,
                    'hz': round(hz, 1),
                    'hz_nominal': elo.hz,
                    'idade': None if idade is None else round(idade, 2),
                    'total': est.total,
                    'publicadores': pubs,
                    'assinantes': subs,
                    'normal_se': elo.normal_se,
                })

        nos = []
        for rotulo, nomes, regra in NOS_ESPERADOS:
            presentes = [n for n in nomes if presente(n, vivos)]
            if regra == 'exclusivo':
                if len(presentes) == 1:
                    status = 'ok'
                elif not presentes:
                    status = 'bad'
                else:
                    status = 'bad'   # os dois de pé: a falha do stack órfão
            elif regra == 'obrigatório':
                status = 'ok' if presentes else 'bad'
            else:
                status = 'ok' if presentes else 'unknown'
            nos.append({'rotulo': rotulo, 'esperados': list(nomes),
                        'presentes': presentes, 'regra': regra,
                        'status': status})

        return {
            'elos': elos,
            'nos': nos,
            'veredito': self._veredito(elos, nos),
            'uptime': round(agora - self._t0, 1),
            # Nós de pé que a topologia não conhece. Um `simulator` órfão
            # sobrevivente de outra sessão aparece aqui antes de aparecer como
            # segunda fonte de /game_data.
            'estado_jogo': self._ctx.estado_jogo or '—',
            # Nomes com "_" na frente são internos do ROS (o daemon do
            # `ros2 cli`, por exemplo) e nunca são o que se procura aqui.
            'outros': sorted(
                n for n in (
                    vivos
                    - {alias for _, ns, _ in NOS_ESPERADOS
                       for n in ns for alias in nomes_de(n)}
                    - {self.get_name()})
                if not n.startswith('_')),
        }

    def _veredito(self, elos, nos):
        """A primeira coisa quebrada na ordem da cadeia, e o que fazer.

        Ordem importa: com a visão morta, TUDO fica vermelho, e listar dez
        falhas esconde a única que interessa. O primeiro elo quebrado na
        direção do fluxo é a causa; o resto é consequência.
        """
        mortos = [n for n in nos if n['status'] == 'bad'
                  and n['regra'] != 'opcional']
        if mortos:
            fonte = next((n for n in mortos if n['regra'] == 'exclusivo'), None)
            if fonte and len(fonte['presentes']) > 1:
                return {'nivel': 'bad', 'onde': 'duas fontes de /game_data',
                        'texto': f'{" e ".join(fonte["presentes"])} estão os dois de pé.',
                        'dica': 'A câmera e o simulador publicam o mesmo tópico '
                                'de propósito, mas um de cada vez. Rode '
                                './tools/stop.sh e suba de novo.'}
            return {'nivel': 'bad', 'onde': mortos[0]['rotulo'],
                    'texto': f'não está rodando '
                             f'({" ou ".join(mortos[0]["esperados"])}).',
                    'dica': 'Confira o launch. ./tools/stop.sh --check mostra '
                            'o que está de pé e em que porta.'}

        ruins = sorted([e for e in elos if e['status'] == 'bad'],
                       key=lambda e: e['etapa'])
        if ruins:
            e = ruins[0]
            return {'nivel': 'bad', 'onde': e['topico'], 'texto': e['detalhe'],
                    'dica': e['dica'] or 'Os elos abaixo deste provavelmente '
                                         'são consequência, não causa.'}

        avisos = sorted([e for e in elos if e['status'] == 'warn'],
                        key=lambda e: e['etapa'])
        if avisos:
            e = avisos[0]
            return {'nivel': 'warn', 'onde': e['topico'], 'texto': e['detalhe'],
                    'dica': e['dica']}

        return {
            'nivel': 'ok', 'onde': 'da câmera ao /motorVelocities',
            'texto': 'a cadeia inteira está passando.',
            'dica': 'Se o robô ainda não anda, o problema está depois do '
                    'último tópico: serial, rádio, ID do robô, alimentação do '
                    'motor ou mecânica. Pare o stack e use ./tools/debug_panel.py.',
        }

    # ── Web ────────────────────────────────────────────────────────────────

    def _start_web_server(self):
        threading.Thread(target=self._run_web_server, daemon=True).start()

    def _run_web_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        app = web.Application()
        app.router.add_get('/', self._serve('index.html'))
        app.router.add_get('/vss.css', self._serve('vss.css'))
        app.router.add_get('/fonts/{name}', self._serve_font)
        # Sondagem e não WebSocket: o painel é de diagnóstico e precisa
        # sobreviver a reconexão sem lógica nenhuma. Um GET a cada 500 ms de
        # um JSON de poucos KB não é o gargalo de nada aqui, e um socket que
        # cai calado é justamente o tipo de falha que este painel existe para
        # não ter.
        app.router.add_get('/estado', self._serve_estado)

        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, '0.0.0.0', self.port)

        try:
            loop.run_until_complete(site.start())
        except OSError as exc:
            self.get_logger().fatal(
                f'Não consegui abrir a porta {self.port}: {exc}\n'
                f'Provavelmente já há um painel rodando.')
            os._exit(1)

        loop.run_forever()

    async def _serve_estado(self, request):
        return web.json_response(self._montar())

    def _serve(self, filename):
        async def handler(request):
            share = get_package_share_directory('diagnostics')
            return web.FileResponse(os.path.join(share, 'web', filename))
        return handler

    async def _serve_font(self, request):
        """Fonte auto-hospedada, uma por vez.

        Não usa `add_static`: com `colcon build --symlink-install` cada .woff2
        instalado é um symlink para o repositório, e o `add_static` do aiohttp
        recusa servir através de symlink. O sintoma é cruel — 404 em toda
        fonte, a tela cai calada para a fonte do sistema e nada aparece no log.
        """
        base = os.path.join(
            get_package_share_directory('diagnostics'), 'web', 'fonts')
        alvo = os.path.normpath(os.path.join(base, request.match_info['name']))

        if os.path.dirname(alvo) != os.path.normpath(base) or not os.path.isfile(alvo):
            raise web.HTTPNotFound()

        return web.FileResponse(
            alvo, headers={'Content-Type': 'font/woff2',
                           'Cache-Control': 'public, max-age=86400'})


def main(args=None):
    rclpy.init(args=args)
    node = FlowPanel()
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
