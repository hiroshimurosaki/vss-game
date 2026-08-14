#!/usr/bin/env python3
"""Painel de diagnóstico da cadeia navegador → ponte → rádio → robô → motor.

    ./tools/debug_panel.py
    # abre http://localhost:8061

O `radio_console.py` responde "o robô anda?". Este responde a pergunta seguinte,
que é a caro: **onde a informação morre?** Ele existe porque quase toda falha
dessa cadeia tem o mesmo sintoma — robô parado, nenhum erro em lugar nenhum — e
por isso o sintoma não aponta para a causa.

A diferença em relação a tudo que já existe aqui: este script abre as **duas**
seriais, a da ponte e a do robô. Com o robô no USB da bancada, dá para ouvir o
lado que recebe em vez de inferir por auto-ACK. É o que permite separar:

    a ponte não recebeu os bytes        → problema no PC/serial
    a ponte recebeu e mandou            → mas nada chega no robô: rádio
    o robô recebeu com checksum ruim    → ruído ou desalinho de stream
    o robô recebeu e descartou por ID   → MY_ROBOT_ID != id enviado
    o robô aceitou e aplicou PWM        → e não gira: motor, bateria, mecânica

O último elo é o único que o software não vê: PWM aplicado com roda parada é
alimentação de motor (VM do TB6612), fio solto ou mecânica travada. Quando o
painel chega nesse estado ele diz isso, em vez de deixar você procurando rádio.

PRECISA DOS FIRMWARES EM MODO DEBUG, senão os dois lados ficam mudos e o painel
não tem o que ler:

    arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 \\
        --build-property compiler.cpp.extra_flags=-DDEBUG_RADIO=1 firmware/robot_rx
    arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 \\
        --build-property compiler.cpp.extra_flags=-DDEBUG_TX=1 firmware/tx_bridge

Debug ligado deixa o loop mais lento e pode perder pacote: é para diagnosticar,
não para jogar. Regrave sem o `-D` antes da feira.

Toda sessão é gravada em `~/.vss-game/logs/` (JSONL), para o caso de a falha
aparecer quando ninguém está olhando a tela — `--no-log` desliga. O que fica
gravado é mais do que a tela mostra: veja o `Recorder`. Para ler depois:

    ls -t ~/.vss-game/logs/ | head                       # a sessão mais recente
    jq -r 'select(.k=="raw") | "\\(.h) \\(.src) \\(.line)"' SESSAO.jsonl
    jq -r 'select(.k=="snap") | "\\(.h) rx_ok=\\(.numbers.rx_ok) \\(.verdict)"' SESSAO.jsonl

Só stdlib: nada de pyserial, nada de aiohttp.
"""

import argparse
import collections
import fcntl
import glob
import http.server
import json
import os
import re
import socketserver
import struct
import sys
import termios
import threading
import time

START_BYTE = 0x14
PACKET_FMT = '<BffiB'
PACKET_SIZE = struct.calcsize(PACKET_FMT)
assert PACKET_SIZE == 14, f'pacote deveria ter 14 bytes, tem {PACKET_SIZE}'


def build_packet(robot_id: int, left: float, right: float) -> bytes:
    """Mesmo layout e checksum do firmware. Ver README, 'Protocolo do rádio'."""
    left = max(-1.0, min(1.0, left))
    right = max(-1.0, min(1.0, right))
    body = struct.pack('<ffi', left, right, robot_id)
    checksum = 0
    for b in body:
        checksum ^= b
    return struct.pack('<B', START_BYTE) + body + struct.pack('<B', checksum)


def open_serial(port: str, baud: int = 115200) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, f'B{baud}')
    attrs[4] = attrs[5] = speed
    attrs[2] = (attrs[2] | termios.CLOCAL | termios.CREAD) & ~termios.CRTSCTS
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


# ── Descoberta das placas ────────────────────────────────────────────────────
# As duas placas são CH340 sem número de série, então `/dev/serial/by-id/` dá o
# mesmo nome para as duas e ttyUSB0/ttyUSB1 dependem da ordem de plugar. Apontar
# para a placa errada já custou tempo duas vezes, e o sintoma é o mesmo de rádio
# quebrado: nada acontece, nenhum erro. Quem responde quem é quem é o firmware.

BANNER_WINDOW = 3.5

# Baixar e subir o DTR reseta o Nano. Abrir a porta normalmente já faz isso, mas
# só na *transição* do sinal: se o DTR já estava alto, a abertura não reseta nada
# e o banner nunca vem. Medido: a detecção falha exatamente assim, devolvendo
# banner vazio. Pulsar explicitamente torna determinístico.
TIOCMBIC, TIOCMBIS, TIOCM_DTR = 0x5417, 0x5416, 0x002


def pulse_dtr(fd: int):
    fcntl.ioctl(fd, TIOCMBIC, struct.pack('I', TIOCM_DTR))
    time.sleep(0.2)
    fcntl.ioctl(fd, TIOCMBIS, struct.pack('I', TIOCM_DTR))


def read_banner(fd: int, window: float = BANNER_WINDOW) -> str:
    """Coleta o que a placa imprime no boot, resetando-a para isso."""
    pulse_dtr(fd)
    buf = b''
    t0 = time.time()
    while time.time() - t0 < window:
        try:
            buf += os.read(fd, 256)
        except (BlockingIOError, OSError):
            pass
        time.sleep(0.05)
    return buf.decode('ascii', 'replace')


def classify(banner: str):
    """('ponte'|'robo'|'antigo'|'?', detalhe) a partir do banner de boot."""
    if 'tx_bridge' in banner:
        return 'ponte', 'radio FAIL' not in banner
    if 'robot_rx' in banner:
        m = re.search(r'ID:\s*(-?\d+)', banner)
        return 'robo', int(m.group(1)) if m else None
    if 'Comandos:' in banner or '1=frente' in banner:
        return 'antigo', 'sketch de menu — grave o firmware/tx_bridge'
    if 'radio' in banner:
        return 'antigo', 'RX sem filtro de ID (franky/Rx_Arduino)'
    return '?', banner.strip()[:60]


#: ttyUSB é CH340 (Nano, Uno clone); ttyACM é o ATmega16U2 do Uno oficial, que a
#: ponte também pode ser. Varrer só ttyUSB deixava uma ponte em Uno oficial
#: invisível para o painel — que reporta isso como "ponte não encontrada", ou
#: seja, igualzinho a rádio quebrado. O reset por DTR funciona nos dois.
PORTAS_GLOB = ('/dev/ttyUSB*', '/dev/ttyACM*')


def portas_seriais():
    return sorted(p for padrao in PORTAS_GLOB for p in glob.glob(padrao))


def detect(baud: int):
    """Abre toda serial de placa, lê o banner e diz quem é a ponte e quem é o robô."""
    found = {}
    for port in portas_seriais():
        try:
            fd = open_serial(port, baud)
        except OSError as exc:
            print(f'  {port}: não abriu ({exc})', file=sys.stderr)
            continue
        kind, detail = classify(read_banner(fd))
        print(f'  {port}: {kind} ({detail})')
        found[port] = (fd, kind, detail)
    return found


# ── Estado compartilhado ─────────────────────────────────────────────────────

class Rate:
    """Contador de eventos por segundo numa janela deslizante.

    Taxa, não total: o que interessa é "está acontecendo agora", porque um total
    alto de dez minutos atrás parece saudável e não é.
    """

    def __init__(self, window=1.0):
        self.window = window
        self.hits = collections.deque()
        self.total = 0

    def hit(self, n=1):
        now = time.time()
        for _ in range(n):
            self.hits.append(now)
        self.total += n

    def value(self):
        cut = time.time() - self.window
        while self.hits and self.hits[0] < cut:
            self.hits.popleft()
        return len(self.hits) / self.window


class State:
    def __init__(self):
        self.lock = threading.Lock()

        # o que o navegador pediu
        self.robot_id = 1
        self.left = 0.0
        self.right = 0.0
        self.last_cmd = 0.0
        self.stale = True

        # servidor → ponte
        self.tx = Rate()
        self.tx_errors = 0

        # ponte → ar (linha "TX id N", exige DEBUG_TX=1)
        self.bridge_banner = ''
        self.bridge_tx = Rate()
        self.bridge_speaks = False        # a ponte já disse qualquer coisa?
        # ACK confirmado × sem confirmação. Contados separados porque o ACK é
        # respondido pelo HARDWARE do nRF24 do robô: é o único sinal de vida que
        # sobrevive ao robô sair do USB. Sem isto o painel fica cego justamente
        # no teste em que o robô anda solto, que é o teste que importa.
        self.bridge_ack = Rate()
        self.bridge_noack = Rate()

        # ar → robô (linhas do robot_rx com DEBUG_RADIO=1)
        self.robot_banner = ''
        self.robot_id_fw = None
        self.robot_radio_ok = None
        self.rx_any = Rate()              # qualquer linha = rádio chegando
        self.rx_ok = Rate()               # passou start byte, checksum e ID
        self.rx_checksum = Rate()
        self.rx_startbyte = Rate()
        self.rx_alheio = Rate()
        self.last_alheio_id = None
        self.pwm_a = None
        self.pwm_b = None
        self.rx_m1 = None
        self.rx_m2 = None
        self.rx_at = 0.0
        # A linha crua do último pacote aceito, e quantas linhas de aceite
        # vieram sem os campos esperados. Sem isto o painel mostra um número
        # extraído sem mostrar de onde veio, e um número errado fica plausível.
        self.last_ok_line = ''
        self.last_ok_at = 0.0
        self.rx_malformed = Rate()
        # Endereços que mandaram comando na última janela. Duas abas abertas (ou
        # uma aba mais um script de teste) disputam o mesmo estado e os comandos
        # se alternam — o que parece pacote fantasma vindo do rádio.
        self.clients = {}

        self.log = collections.deque(maxlen=120)

        # override temporário para o teste de força
        self.boost_until = 0.0

    def note(self, source, line):
        self.log.append({'t': time.strftime('%H:%M:%S'), 'src': source, 'line': line[:90]})


# ── Gravação em disco ────────────────────────────────────────────────────────

LOG_DIR_DEFAULT = os.path.expanduser('~/.vss-game/logs')


class Recorder:
    """Grava a sessão em JSONL para autópsia depois que o painel morrer.

    O `st.log` da tela é uma janela de 120 linhas que some junto com o processo:
    serve para olhar agora, não para diagnosticar depois. E para caber na tela
    ele **descarta** justamente o que acontece a 30 Hz (`OK |`, `TX id`,
    `ID ALHEIO`) — que é a série temporal onde a falha intermitente aparece.
    Um arquivo que fosse espelho da tela herdaria os dois defeitos.

    Aqui é o contrário: linha crua, toda ela, dos dois lados, mais um retrato
    dos contadores e do veredito por segundo. Quem ler isto depois quer
    responder "o que mudou no instante em que parou"; para isso, o que a tela
    filtra é o dado.

    Um registro por linha, distinguidos pela chave `k`:

        meta   uma vez, na abertura: portas, argv, hora de início
        raw    uma por linha de serial: {t, h, src, line}
        snap   uma por segundo: os mesmos contadores que a tela mostra
        end    uma vez, no encerramento limpo (a ausência dele já é informação:
               significa que o painel foi morto, não que parou)

    Falha de escrita nunca derruba o painel. O painel é a ferramenta de
    diagnóstico: perder o log é ruim, perder o diagnóstico é pior.
    """

    def __init__(self, directory=LOG_DIR_DEFAULT, max_bytes=64 << 20,
                 keep_sessions=10, keep_parts=2):
        self.dir = directory
        self.max_bytes = max_bytes
        self.keep_parts = keep_parts
        self.lock = threading.Lock()
        self.fh = None
        self.written = 0
        self.part = 0
        self.failed = False
        self.snapshotter = None
        self.running = True
        self._meta = None

        os.makedirs(self.dir, exist_ok=True)
        self.base = 'debug_panel-' + time.strftime('%Y%m%d-%H%M%S')
        self._prune_sessions(keep_sessions)
        self._roll()

    # -- arquivos --------------------------------------------------------

    def _path(self, part):
        return os.path.join(self.dir, f'{self.base}-part{part}.jsonl')

    def _roll(self):
        """Abre a próxima parte e joga fora as partes velhas desta sessão.

        Rotação em vez de parar no limite: numa falha intermitente o trecho que
        importa é o do fim, e um arquivo que para de crescer perde exatamente
        esse trecho. O custo é o teto virar `keep_parts` × `max_bytes`.

        O `meta` é reescrito no topo de cada parte. Sem isso a sessão longa — a
        que mais tem chance de ser lida depois — perde o cabeçalho junto com a
        parte 1, e sobra um arquivo de linhas sem dizer de que porta vieram.
        """
        if self.fh:
            self.fh.close()
        self.part += 1
        self.fh = open(self._path(self.part), 'w', buffering=1)
        self.written = 0
        if self._meta:
            now = time.time()
            head = dict(self._meta, parte=self.part, continuacao=True,
                        t=round(now, 3),
                        h=time.strftime('%H:%M:%S', time.localtime(now)))
            line = json.dumps(head, ensure_ascii=False) + '\n'
            self.fh.write(line)
            self.written += len(line)
        for old in range(1, self.part - self.keep_parts + 1):
            try:
                os.remove(self._path(old))
            except OSError:
                pass

    def _prune_sessions(self, keep):
        """Mantém só as `keep` sessões mais recentes. O nome ordena por tempo."""
        sessions = set()
        for path in glob.glob(os.path.join(self.dir, 'debug_panel-*.jsonl')):
            name = os.path.basename(path)
            sessions.add(name.split('-part')[0])
        for stale in sorted(sessions)[:-keep] if len(sessions) > keep else []:
            for path in glob.glob(os.path.join(self.dir, stale + '-part*.jsonl')):
                try:
                    os.remove(path)
                except OSError:
                    pass

    @property
    def path(self):
        return self._path(self.part)

    # -- escrita ---------------------------------------------------------

    def _write(self, rec):
        if self.failed:
            return
        now = time.time()
        rec['t'] = round(now, 3)
        rec['h'] = time.strftime('%H:%M:%S', time.localtime(now)) + f'{now % 1:.3f}'[1:]
        line = json.dumps(rec, ensure_ascii=False) + '\n'
        with self.lock:
            try:
                self.fh.write(line)
                self.written += len(line)
                if self.written >= self.max_bytes:
                    self._roll()
            except OSError as exc:
                self.failed = True
                print(f'AVISO: gravação do log parou ({exc}); o painel segue.',
                      file=sys.stderr)

    def meta(self, **kw):
        rec = dict(k='meta', parte=self.part, **kw)
        self._write(rec)
        with self.lock:
            self._meta = {k: v for k, v in rec.items() if k not in ('t', 'h')}

    def raw(self, src, line):
        self._write({'k': 'raw', 'src': src, 'line': line})

    def start_snapshots(self, produce, period=1.0):
        """Chama `produce()` a cada `period` s e grava o que voltar como `snap`.

        A política de o-que-gravar fica com quem chamou, de propósito: o retrato
        tem de ser o **mesmo** que a tela mostra, e ele nasce lá.
        """
        def run():
            while self.running:
                time.sleep(period)
                if not self.running:
                    return
                try:
                    self._write(dict(k='snap', **produce()))
                except Exception as exc:                       # noqa: BLE001
                    self._write({'k': 'snap_erro', 'erro': repr(exc)})
        self.snapshotter = threading.Thread(target=run, daemon=True)
        self.snapshotter.start()

    def close(self, motivo='fim'):
        self.running = False
        self._write({'k': 'end', 'motivo': motivo})
        with self.lock:
            try:
                self.fh.close()
            except OSError:
                pass


# ── Threads de serial ────────────────────────────────────────────────────────

class Sender(threading.Thread):
    """Manda o comando corrente a taxa fixa e zera se o navegador sumir.

    Taxa fixa porque o `COMMAND_TIMEOUT` do robô é 1 s: sem fluxo contínuo ele
    para sozinho no meio da manobra. O watchdog fica aqui, no processo que segura
    a serial, porque é o único que continua vivo se o navegador travar.
    """

    def __init__(self, fd, state, rate_hz=30.0, watchdog_s=0.3):
        super().__init__(daemon=True)
        self.fd = fd
        self.st = state
        self.period = 1.0 / rate_hz
        self.watchdog = watchdog_s
        self.running = True
        self.last_packet = b''
        self.view = {}

    def run(self):
        while self.running:
            st = self.st
            with st.lock:
                st.stale = (time.time() - st.last_cmd) > self.watchdog
                boosting = time.time() < st.boost_until
                if st.stale:
                    left = right = 0.0
                elif boosting:
                    left = right = 1.0
                else:
                    left, right = st.left, st.right
                rid = st.robot_id
            pkt = build_packet(rid, left, right)
            try:
                os.write(self.fd, pkt)
                with st.lock:
                    st.tx.hit()
                    self.last_packet = pkt
                    self.view = {'rid': rid, 'left': left, 'right': right,
                                 'stale': st.stale, 'state_obj': id(st),
                                 'ticks': self.view.get('ticks', 0) + 1}
            except OSError:
                with st.lock:
                    st.tx_errors += 1
            time.sleep(self.period)


class LineReader(threading.Thread):
    """Lê linhas de uma serial e entrega ao parser. Nunca escreve.

    O `tap` recebe a linha **antes** do parser e sem filtro nenhum. Fica aqui, e
    não dentro dos handlers, porque os handlers retornam cedo nos casos de 30 Hz
    — gravar lá herdaria os furos que existem para poupar a tela.
    """

    def __init__(self, fd, handler, tap=None):
        super().__init__(daemon=True)
        self.fd = fd
        self.handler = handler
        self.tap = tap
        self.running = True

    def run(self):
        buf = b''
        while self.running:
            try:
                data = os.read(self.fd, 512)
            except (BlockingIOError, OSError):
                data = b''
            if not data:
                time.sleep(0.02)
                continue
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('utf-8', 'replace').strip()
                if text:
                    if self.tap:
                        self.tap(text)
                    self.handler(text)
            if len(buf) > 1024:
                buf = buf[-256:]


PWM_RE = re.compile(r'PWM_A:\s*(-?\d+).*PWM_B:\s*(-?\d+)')
M_RE = re.compile(r'M1:\s*(-?[\d.]+).*M2:\s*(-?[\d.]+)')


def bridge_line(st):
    def handle(line):
        with st.lock:
            st.bridge_speaks = True
            if line.startswith('TX id'):
                st.bridge_tx.hit()
                # 'SEM ACK' termina em 'ACK': testar o negativo primeiro, senão
                # a falha é contada como sucesso e o painel mente para o lado
                # perigoso.
                if line.endswith('SEM ACK'):
                    st.bridge_noack.hit()
                elif line.endswith('ACK'):
                    st.bridge_ack.hit()
                return                    # 30 Hz disso encheria o log
            if 'tx_bridge' in line:
                st.bridge_banner = line
            st.note('ponte', line)
    return handle


def robot_line(st):
    """Traduz o log do robot_rx em contadores.

    Cada ramo aqui corresponde a um `return` do `loop()` do firmware, e é essa
    correspondência que faz o painel apontar a causa em vez do sintoma.
    """
    def handle(line):
        with st.lock:
            st.rx_any.hit()
            st.rx_at = time.time()

            if line.startswith('OK |'):
                st.rx_ok.hit()
                st.last_ok_line = line
                st.last_ok_at = time.time()
                pwm = PWM_RE.search(line)
                mot = M_RE.search(line)
                if pwm:
                    st.pwm_a, st.pwm_b = int(pwm.group(1)), int(pwm.group(2))
                if mot:
                    st.rx_m1, st.rx_m2 = float(mot.group(1)), float(mot.group(2))
                if not (pwm and mot):
                    # Linha de aceite truncada. Se isto sobe, o número mostrado
                    # acima é de um pacote antigo e não vale nada.
                    st.rx_malformed.hit()
                return                    # 30 Hz disso encheria o log
            if line.startswith('CHECKSUM FAIL'):
                st.rx_checksum.hit()
            elif line.startswith('START BYTE'):
                st.rx_startbyte.hit()
            elif line.startswith('ID ALHEIO'):
                st.rx_alheio.hit()
                m = re.search(r'(-?\d+)', line)
                if m:
                    st.last_alheio_id = int(m.group(1))
                return                    # também é 30 Hz quando acontece
            elif line.startswith('robot_rx'):
                st.robot_banner = line
                m = re.search(r'ID:\s*(-?\d+)', line)
                if m:
                    st.robot_id_fw = int(m.group(1))
            elif line.startswith('radio '):
                st.robot_radio_ok = ('OK' in line)
            st.note('robô', line)
    return handle


# ── O diagnóstico ────────────────────────────────────────────────────────────
# A ordem dos elos é a ordem da cadeia. O primeiro elo quebrado é a causa; tudo
# depois dele está quebrado por consequência e não deve ser investigado.

OK, BAD, WARN, UNKNOWN = 'ok', 'bad', 'warn', 'unknown'


def diagnose(st, robot_present, bridge_present):
    with st.lock:
        # Calculado aqui, não lido de st.stale: aquele flag só é atualizado pelo
        # thread de envio a cada 33 ms, e o POST que acabou de chegar leria o
        # valor velho — o painel acusaria "sem comando" no instante em que o
        # comando chegou.
        stale = (time.time() - st.last_cmd) > 0.35
        tx_rate = st.tx.value()
        tx_errors = st.tx_errors
        bridge_speaks = st.bridge_speaks
        bridge_rate = st.bridge_tx.value()
        rx_any = st.rx_any.value()
        rx_ok = st.rx_ok.value()
        rx_cks = st.rx_checksum.value()
        rx_sb = st.rx_startbyte.value()
        rx_alheio = st.rx_alheio.value()
        alheio_id = st.last_alheio_id
        pwm_a, pwm_b = st.pwm_a, st.pwm_b
        rid = st.robot_id
        fw_id = st.robot_id_fw
        radio_ok = st.robot_radio_ok
        left, right = st.left, st.right
        commanding = (abs(left) > 0.01 or abs(right) > 0.01
                      or time.time() < st.boost_until)
        clients = len(st.clients)

    links = []

    def add(name, status, detail, hint=''):
        links.append({'name': name, 'status': status, 'detail': detail, 'hint': hint})

    # 1. navegador → servidor
    add('navegador → servidor',
        BAD if stale else OK,
        'sem comando há mais de 0,3 s' if stale else 'recebendo comando',
        'A aba está aberta e em foco? Solte e pressione uma tecla.' if stale else '')

    # 2. servidor → ponte
    add('servidor → ponte (serial)',
        BAD if tx_rate < 1 else (WARN if tx_errors else OK),
        f'{tx_rate:.0f} pacotes/s, {tx_errors} erros',
        'Erro de escrita na serial: a placa foi desplugada?' if tx_errors else '')

    # 3. ponte → rádio
    if not bridge_present:
        add('ponte → rádio', UNKNOWN, 'ponte não conectada',
            'Plugue a ponte e reinicie o painel.')
    elif not bridge_speaks:
        add('ponte → rádio', UNKNOWN, 'ponte muda (sem DEBUG_TX)',
            'Regrave com -DDEBUG_TX=1 para ver a ponte confirmar cada envio.')
    else:
        add('ponte → rádio',
            OK if bridge_rate > 1 else BAD,
            f'{bridge_rate:.0f} repasses/s' if bridge_rate > 1
            else 'a ponte não está repassando',
            '' if bridge_rate > 1
            else 'A ponte recebe bytes mas não valida o checksum: '
                 'baud errado ou stream desalinhado.')

    # 4. rádio → robô
    if not robot_present:
        add('rádio → robô', UNKNOWN, 'robô não está no USB',
            'Este elo só é observável com o robô plugado na bancada.')
    elif radio_ok is False:
        add('rádio → robô', BAD, 'o nRF24 do robô não responde no SPI',
            'radio FAIL no boot: fiação CE=D6/CSN=D10, 3V3 e capacitor.')
    else:
        add('rádio → robô',
            OK if rx_any > 1 else BAD,
            f'{rx_any:.0f} pacotes/s chegando' if rx_any > 1 else 'nada chega pelo rádio',
            '' if rx_any > 1
            else 'Canal, data rate e auto-ACK precisam bater nos dois lados. '
                 'Confira também alimentação do módulo e distância.')

    # 5. o robô aceita o pacote
    if not robot_present:
        add('robô aceita o pacote', UNKNOWN, 'robô não está no USB')
    elif rx_alheio > 1 and rx_ok < 1:
        add('robô aceita o pacote', BAD,
            f'descartado por ID: chegou id={alheio_id}, este robô é {fw_id}',
            f'Selecione o robô {fw_id} no painel, ou regrave com '
            f'MY_ROBOT_ID {rid}.')
    elif rx_cks > 1 and rx_ok < 1:
        add('robô aceita o pacote', BAD, f'{rx_cks:.0f} checksums ruins/s',
            'Pacote corrompido no ar ou struct diferente entre os três lugares '
            'do protocolo. Ver contrato 2 no docs/ARQUITETURA.md.')
    elif rx_sb > 1 and rx_ok < 1:
        add('robô aceita o pacote', BAD, f'{rx_sb:.0f} start bytes inválidos/s',
            'START_BYTE diferente entre ROS, ponte e robô.')
    elif rx_any <= 1:
        add('robô aceita o pacote', UNKNOWN, 'sem pacote para julgar')
    else:
        add('robô aceita o pacote', OK if rx_ok > 1 else WARN,
            f'{rx_ok:.0f} aceitos/s' + (f', {rx_cks + rx_sb + rx_alheio:.0f} descartados/s'
                                        if (rx_cks + rx_sb + rx_alheio) > 1 else ''))

    # 6. robô → PWM
    if not robot_present or rx_ok <= 1:
        add('robô → PWM', UNKNOWN, 'sem pacote aceito')
    elif not commanding:
        add('robô → PWM', UNKNOWN, 'comando é zero — nada a aplicar',
            'Pressione uma direção ou use o teste de força.')
    elif (pwm_a or 0) == 0 and (pwm_b or 0) == 0:
        add('robô → PWM', WARN, f'PWM 0/0 com comando {left:+.2f}/{right:+.2f}',
            'Abaixo de 25 de 255 (~0,10) o firmware zera de propósito: '
            'nessa faixa o motor só chia e esquenta. Aumente a força.')
    else:
        add('robô → PWM', OK, f'PWM {pwm_a}/{pwm_b} aplicado')

    # 7. PWM → roda. O software não vê; é aqui que o painel para de responder e
    # passa a perguntar.
    if (pwm_a or 0) or (pwm_b or 0):
        add('PWM → roda gira', UNKNOWN, 'o software não vê este elo',
            'PWM saindo e roda parada = alimentação de motor (VM do TB6612), '
            'STBY em D2, fio de motor solto ou mecânica travada. '
            'O USB alimenta o Nano e o rádio, mas NÃO os motores.')
    else:
        add('PWM → roda gira', UNKNOWN, 'sem PWM para julgar')

    if clients > 1:
        links.insert(0, {
            'name': 'quem está mandando', 'status': BAD,
            'detail': f'{clients} clientes mandando comando ao mesmo tempo',
            'hint': 'Duas abas, ou uma aba mais um script de teste: os comandos '
                    'se alternam e a medição não vale. Feche uma.'})

    first_bad = next((l for l in links if l['status'] == BAD), None)
    if first_bad:
        verdict = {'level': BAD, 'where': first_bad['name'],
                   'text': first_bad['detail'], 'hint': first_bad['hint']}
    elif any(l['status'] == WARN for l in links):
        w = next(l for l in links if l['status'] == WARN)
        verdict = {'level': WARN, 'where': w['name'],
                   'text': w['detail'], 'hint': w['hint']}
    elif all(l['status'] in (OK, UNKNOWN) for l in links) and rx_ok > 1:
        verdict = {'level': OK, 'where': 'cadeia inteira',
                   'text': 'o robô está recebendo e aplicando PWM',
                   'hint': 'Se a roda não gira com PWM saindo, o problema é '
                           'elétrico ou mecânico, não de software.'}
    else:
        verdict = {'level': UNKNOWN, 'where': 'aguardando',
                   'text': 'mande um comando para medir a cadeia', 'hint': ''}
    return links, verdict


PAGE = """<!DOCTYPE html>
<html lang="pt-BR"><head><meta charset="utf-8">
<title>Painel de diagnóstico — VSS</title>
<style>
 :root { color-scheme: dark; }
 * { box-sizing: border-box; }
 body { margin:0; background:#12141a; color:#e6e8ee;
        font:14px/1.55 system-ui,-apple-system,Segoe UI,sans-serif; }
 header { padding:12px 18px; background:#1b1e26; border-bottom:1px solid #2b3040;
          display:flex; gap:16px; align-items:center; flex-wrap:wrap; }
 h1 { font-size:15px; margin:0; font-weight:600; }
 main { padding:18px; display:grid; gap:18px;
        grid-template-columns:minmax(420px,1.4fr) minmax(300px,1fr); align-items:start; }
 @media (max-width:900px) { main { grid-template-columns:1fr; } }
 .card { background:#1b1e26; border:1px solid #2b3040; border-radius:8px; padding:14px 16px; }
 .card h2 { font-size:12px; margin:0 0 12px; text-transform:uppercase;
            letter-spacing:.07em; color:#8f97ad; font-weight:600; }
 .verdict { border-left:4px solid #39405f; padding:12px 16px; border-radius:6px;
            background:#0e1015; }
 .verdict .big { font-size:17px; font-weight:600; }
 .verdict.ok { border-color:#4ec98a; } .verdict.bad { border-color:#e0614f; }
 .verdict.warn { border-color:#e0913a; } .verdict.unknown { border-color:#5b6480; }
 .chain { display:flex; flex-direction:column; gap:0; }
 .link { display:grid; grid-template-columns:14px 1fr auto; gap:12px;
         padding:10px 0; border-bottom:1px solid #23283a; align-items:start; }
 .link:last-child { border-bottom:none; }
 .dot { width:12px; height:12px; border-radius:50%; margin-top:5px; background:#5b6480; }
 .dot.ok { background:#4ec98a; } .dot.bad { background:#e0614f; }
 .dot.warn { background:#e0913a; } .dot.unknown { background:#3a4056; }
 .lname { font-weight:600; font-size:13px; }
 .ldetail { color:#b9c0d4; font-size:12.5px; }
 .lhint { color:#8f97ad; font-size:12px; margin-top:3px; font-style:italic; }
 .badge { font-size:11px; color:#8f97ad; text-transform:uppercase; letter-spacing:.05em; }
 table { border-collapse:collapse; font-size:13px; width:100%; }
 td { padding:3px 0; } td.k { color:#8f97ad; } td.v { text-align:right;
      font-variant-numeric:tabular-nums; }
 code { background:#0e1015; padding:2px 7px; border-radius:4px; font-size:12px; }
 .ok { color:#4ec98a; } .warn { color:#e0913a; } .bad { color:#e0614f; }
 .muted { color:#8f97ad; }
 .hint { color:#8f97ad; font-size:12px; margin:8px 0 0; }
 button { background:#2b3040; color:#e6e8ee; border:none; border-radius:6px;
          padding:9px 14px; font-size:14px; cursor:pointer; }
 button:hover { filter:brightness(1.25); }
 button.act { background:#2f6df6; } button.stop { background:#c0392b; font-weight:600; }
 select { background:#12141a; color:#e6e8ee; border:1px solid #2b3040;
          border-radius:6px; padding:7px 10px; font-size:14px; }
 .pad { display:grid; grid-template-columns:repeat(3,60px);
        grid-template-rows:repeat(2,60px); gap:6px; }
 .pad button { font-size:19px; } .pad .sp { visibility:hidden; }
 .row { display:flex; align-items:center; gap:10px; margin:9px 0; }
 .row label { width:58px; font-size:12px; color:#8f97ad; }
 input[type=range] { flex:1; }
 #log { font:11.5px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;
        max-height:190px; overflow:auto; background:#0e1015; border-radius:6px;
        padding:8px 10px; }
 #log div { white-space:pre-wrap; }
</style></head><body>
<header>
  <h1>Painel de diagnóstico</h1>
  <span>enviar para o robô <select id="rid">
    <option>0</option><option selected>1</option><option>2</option><option>3</option>
  </select></span>
  <span class="badge" id="ports">—</span>
  <span style="flex:1"></span>
  <button class="stop" id="estop">PARAR (espaço)</button>
</header>
<main>
 <div style="display:flex;flex-direction:column;gap:18px">
  <div class="card">
    <h2>Onde a informação morre</h2>
    <div class="verdict unknown" id="verdict">
      <div class="big" id="v_where">—</div>
      <div id="v_text" class="muted">—</div>
      <div id="v_hint" class="lhint"></div>
    </div>
    <div class="chain" id="chain" style="margin-top:14px"></div>
  </div>

  <div class="card">
    <h2>Log das duas placas</h2>
    <div id="log"></div>
    <p class="hint">As linhas de 30 Hz (pacote aceito e repasse da ponte) não
    entram aqui de propósito — elas viram taxa nos elos acima. O que aparece são
    os eventos raros, que é onde está a informação.</p>
  </div>
 </div>

 <div style="display:flex;flex-direction:column;gap:18px">
  <div class="card">
    <h2>Dirigir</h2>
    <div class="pad">
      <div class="sp"></div><button data-k="w">▲</button><div class="sp"></div>
      <button data-k="a">◀</button><button data-k="s">▼</button><button data-k="d">▶</button>
    </div>
    <p class="hint">WASD ou setas. <b>Soltar para.</b> Q e E giram no lugar.</p>
    <div class="row"><label>força</label>
      <input type="range" id="speed" min="0" max="100" value="60">
      <output id="speedv">0.60</output></div>
    <button class="act" id="boost">Teste de força (1 s a 100%)</button>
    <p class="hint">Manda 1,0/1,0 por um segundo. Serve para tirar a zona morta
    da equação: se nem assim sai PWM, o problema não é força de comando.</p>
  </div>

  <div class="card">
    <h2>Números</h2>
    <table>
      <tr><td class="k">enviando para o id</td><td class="v" id="n_rid">—</td></tr>
      <tr><td class="k">robô responde como</td><td class="v" id="n_fwid">—</td></tr>
      <tr><td class="k">pacotes/s ao serial</td><td class="v" id="n_tx">—</td></tr>
      <tr><td class="k">repasses/s da ponte</td><td class="v" id="n_br">—</td></tr>
      <tr><td class="k">chegando no robô/s</td><td class="v" id="n_any">—</td></tr>
      <tr><td class="k">aceitos/s</td><td class="v" id="n_ok">—</td></tr>
      <tr><td class="k">descartados por ID/s</td><td class="v" id="n_alh">—</td></tr>
      <tr><td class="k">checksum ruim/s</td><td class="v" id="n_cks">—</td></tr>
      <tr><td class="k">start byte ruim/s</td><td class="v" id="n_sb">—</td></tr>
      <tr><td class="k">M1 / M2 no robô</td><td class="v" id="n_m">—</td></tr>
      <tr><td class="k">PWM A / B</td><td class="v" id="n_pwm">—</td></tr>
      <tr><td class="k">linhas truncadas/s</td><td class="v" id="n_mal">—</td></tr>
      <tr><td class="k">clientes mandando</td><td class="v" id="n_cli">—</td></tr>
    </table>
    <p class="hint">Última linha crua que o robô imprimiu ao aceitar:</p>
    <p><code id="n_okline">—</code></p>
    <p class="hint">Pacote que está saindo:</p>
    <p><code id="n_hex">—</code></p>
  </div>
 </div>
</main>
<script>
let speed = 0.6, held = new Set();
// Identidade desta aba, para o painel saber distinguir "duas abas disputando" de
// "um cliente reconectando". Sem isto cada requisição parece um cliente novo.
const CID = 'aba-' + Math.random().toString(36).slice(2, 8);
const KEYS = { w:[1,1], arrowup:[1,1], s:[-1,-1], arrowdown:[-1,-1],
  a:[-0.6,0.6], arrowleft:[-0.6,0.6], d:[0.6,-0.6], arrowright:[0.6,-0.6],
  q:[-1,1], e:[1,-1] };

function wheels() {
  let l = 0, r = 0;
  for (const k of held) { const v = KEYS[k]; if (v) { l += v[0]; r += v[1]; } }
  const m = Math.max(1, Math.abs(l), Math.abs(r));
  return [l/m*speed, r/m*speed];
}

function render(s) {
  const v = document.getElementById('verdict');
  v.className = 'verdict ' + s.verdict.level;
  document.getElementById('v_where').textContent = s.verdict.where;
  document.getElementById('v_text').textContent = s.verdict.text;
  document.getElementById('v_hint').textContent = s.verdict.hint || '';

  document.getElementById('chain').innerHTML = s.links.map(l => `
    <div class="link">
      <div class="dot ${l.status}"></div>
      <div><div class="lname">${l.name}</div>
           <div class="ldetail">${l.detail}</div>
           ${l.hint ? `<div class="lhint">${l.hint}</div>` : ''}</div>
      <div class="badge ${l.status}">${l.status === 'unknown' ? '?' : l.status}</div>
    </div>`).join('');

  const n = s.numbers;
  document.getElementById('n_rid').textContent = n.robot_id;
  document.getElementById('n_fwid').textContent = n.fw_id === null ? '—' : n.fw_id;
  document.getElementById('n_tx').textContent = n.tx.toFixed(0);
  document.getElementById('n_br').textContent = n.bridge === null ? '—' : n.bridge.toFixed(0);
  document.getElementById('n_any').textContent = n.rx_any.toFixed(0);
  document.getElementById('n_ok').textContent = n.rx_ok.toFixed(0);
  document.getElementById('n_alh').textContent = n.rx_alheio.toFixed(0);
  document.getElementById('n_cks').textContent = n.rx_checksum.toFixed(0);
  document.getElementById('n_sb').textContent = n.rx_startbyte.toFixed(0);
  document.getElementById('n_m').textContent =
    n.m1 === null ? '—' : n.m1.toFixed(2) + ' / ' + n.m2.toFixed(2);
  document.getElementById('n_pwm').textContent =
    n.pwm_a === null ? '—' : n.pwm_a + ' / ' + n.pwm_b;
  document.getElementById('n_mal').textContent = n.malformed.toFixed(0);
  const ol = document.getElementById('n_okline');
  if (!n.last_ok_line) { ol.textContent = '—'; ol.className = ''; }
  else if (n.last_ok_age > 1.5) {
    ol.textContent = n.last_ok_line + '   (há ' + n.last_ok_age.toFixed(0) + ' s — dado velho)';
    ol.className = 'warn';
  } else { ol.textContent = n.last_ok_line; ol.className = 'ok'; }
  const cl = document.getElementById('n_cli');
  cl.textContent = n.clients;
  cl.className = 'v ' + (n.clients > 1 ? 'bad' : '');
  document.getElementById('n_hex').textContent = n.hex;
  document.getElementById('ports').textContent = n.ports;
  // Onde o log foi parar precisa estar na tela: quem vai querer o arquivo é
  // quem acabou de ver a falha, e a essa altura o stdout já rolou embora.
  const p = document.getElementById('ports');
  p.title = n.log_file ? 'gravando em ' + n.log_file : 'sessão não está sendo gravada';
  p.style.opacity = n.log_file ? '' : '0.55';

  document.getElementById('log').innerHTML = s.log.slice().reverse()
    .map(e => `<div><span class="muted">${e.t} ${e.src.padEnd(5)}</span> ${e.line}</div>`).join('');
}

async function tick() {
  const [l, r] = wheels();
  try {
    const res = await fetch('/cmd', {method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({cid:CID, id:+document.getElementById('rid').value,
                            left:l, right:r})});
    render(await res.json());
  } catch (e) {
    document.getElementById('v_where').textContent = 'sem servidor';
  }
}
setInterval(tick, 100);

addEventListener('keydown', e => {
  const k = e.key.toLowerCase();
  if (k === ' ') { held.clear(); e.preventDefault(); return; }
  if (KEYS[k]) { held.add(k); e.preventDefault(); }
});
addEventListener('keyup', e => held.delete(e.key.toLowerCase()));
// Perder o foco com a tecla presa deixaria o robô acelerado: o keyup nunca chega.
addEventListener('blur', () => held.clear());

for (const b of document.querySelectorAll('.pad button')) {
  const k = b.dataset.k;
  const on = e => { e.preventDefault(); held.add(k); b.classList.add('act'); };
  const off = () => { held.delete(k); b.classList.remove('act'); };
  b.addEventListener('mousedown', on); b.addEventListener('touchstart', on);
  addEventListener('mouseup', off); b.addEventListener('touchend', off);
  b.addEventListener('mouseleave', off);
}
const sp = document.getElementById('speed');
sp.oninput = () => { speed = sp.value/100;
  document.getElementById('speedv').textContent = speed.toFixed(2); };
document.getElementById('estop').onclick = () => held.clear();
document.getElementById('boost').onclick = () =>
  fetch('/boost', {method:'POST'});
</script></body></html>
"""


def numbers_of(st, sender, ports_label):
    """O retrato dos contadores. Fonte única para a tela e para o log gravado.

    Toma o `st.lock` por conta própria — não chame de dentro de um `with
    st.lock`, que ele não é reentrante. Os campos do `sender` também são
    escritos sob esse lock, por isso são lidos aqui dentro.
    """
    with st.lock:
        return {
            'robot_id': st.robot_id,
            'fw_id': st.robot_id_fw,
            'tx': st.tx.value(),
            'bridge': st.bridge_tx.value() if st.bridge_speaks else None,
            'ack': st.bridge_ack.value(),
            'noack': st.bridge_noack.value(),
            'rx_any': st.rx_any.value(),
            'rx_ok': st.rx_ok.value(),
            'rx_alheio': st.rx_alheio.value(),
            'rx_checksum': st.rx_checksum.value(),
            'rx_startbyte': st.rx_startbyte.value(),
            'm1': st.rx_m1, 'm2': st.rx_m2,
            'pwm_a': st.pwm_a, 'pwm_b': st.pwm_b,
            'last_ok_line': st.last_ok_line,
            'last_ok_age': (time.time() - st.last_ok_at) if st.last_ok_at else None,
            'clients': len(st.clients),
            'malformed': st.rx_malformed.value(),
            'hex': ' '.join(f'{b:02x}' for b in sender.last_packet),
            'ports': ports_label,
            'sender_view': sender.view,
            'state_obj_http': id(st),
        }


class Handler(http.server.BaseHTTPRequestHandler):
    state = None
    sender = None
    recorder = None
    ports_label = ''
    robot_present = False
    bridge_present = False

    def log_message(self, *a):
        pass                      # 10 req/s: o log encheria a tela

    def _json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path not in ('/', '/index.html'):
            self.send_error(404)
            return
        body = PAGE.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        st = self.state
        if self.path == '/boost':
            with st.lock:
                st.boost_until = time.time() + 1.0
                st.last_cmd = time.time()
            self._json({'ok': True})
            return
        if self.path != '/cmd':
            self.send_error(404)
            return

        n = int(self.headers.get('Content-Length', 0))
        try:
            data = json.loads(self.rfile.read(n) or b'{}')
        except json.JSONDecodeError:
            data = {}
        with st.lock:
            st.clients[str(data.get('cid', self.client_address[0]))] = time.time()
            cut = time.time() - 2.0
            st.clients = {k: v for k, v in st.clients.items() if v > cut}
            st.robot_id = int(data.get('id', 1))
            st.left = float(data.get('left', 0.0))
            st.right = float(data.get('right', 0.0))
            st.last_cmd = time.time()

        links, verdict = diagnose(st, self.robot_present, self.bridge_present)
        numbers = numbers_of(st, self.sender, self.ports_label)
        numbers['log_file'] = self.recorder.path if self.recorder else None
        with st.lock:
            log = list(st.log)
        self._json({'links': links, 'verdict': verdict, 'numbers': numbers, 'log': log})


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bridge', help='serial da ponte (default: descobre pelo banner)')
    ap.add_argument('--robot', help='serial do robô (default: descobre pelo banner)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--http-port', type=int, default=8061)
    ap.add_argument('--rate', type=float, default=30.0)
    ap.add_argument('--log-dir', default=LOG_DIR_DEFAULT,
                    help=f'onde gravar a sessão (default: {LOG_DIR_DEFAULT})')
    ap.add_argument('--no-log', action='store_true',
                    help='não gravar nada em disco')
    ap.add_argument('--log-max-mb', type=float, default=64.0,
                    help='tamanho de cada parte antes de rotacionar (default: 64)')
    ap.add_argument('--log-keep', type=int, default=10,
                    help='quantas sessões antigas manter (default: 10)')
    args = ap.parse_args()

    print('descobrindo as placas pelo banner de boot (~3 s por porta)...')
    found = detect(args.baud)

    bridge = robot = None
    for port, (fd, kind, detail) in found.items():
        if kind == 'ponte' and bridge is None:
            bridge = (port, fd)
        elif kind == 'robo' and robot is None:
            robot = (port, fd)
    if args.bridge:
        bridge = (args.bridge, found[args.bridge][0]) if args.bridge in found else None
    if args.robot:
        robot = (args.robot, found[args.robot][0]) if args.robot in found else None

    if bridge is None:
        for port, (fd, kind, detail) in found.items():
            if kind == 'antigo':
                sys.exit(f'{port} tem o firmware errado: {detail}')
        sys.exit('não achei a ponte. Ela está plugada e com o firmware/tx_bridge?')

    label = f'ponte {os.path.basename(bridge[0])}'
    label += f' · robô {os.path.basename(robot[0])}' if robot else ' · robô fora do USB'
    print(f'usando {label}')
    if robot is None:
        print('AVISO: sem o robô no USB o painel não observa o lado que recebe.',
              file=sys.stderr)

    st = State()
    # Um Nano reseta ao abrir a porta e os primeiros pacotes cairiam no
    # bootloader. A descoberta já gastou esse tempo lendo o banner.
    termios.tcflush(bridge[1], termios.TCIFLUSH)

    rec = None
    if not args.no_log:
        try:
            rec = Recorder(args.log_dir, int(args.log_max_mb * (1 << 20)), args.log_keep)
        except OSError as exc:
            print(f'AVISO: não consegui abrir o log em {args.log_dir} ({exc}); '
                  'seguindo sem gravar.', file=sys.stderr)

    sender = Sender(bridge[1], st, rate_hz=args.rate)
    sender.start()
    readers = [LineReader(bridge[1], bridge_line(st),
                          tap=(lambda ln: rec.raw('ponte', ln)) if rec else None)]
    if robot:
        readers.append(LineReader(robot[1], robot_line(st),
                                  tap=(lambda ln: rec.raw('robo', ln)) if rec else None))
    for r in readers:
        r.start()

    if rec:
        rec.meta(inicio=time.strftime('%Y-%m-%d %H:%M:%S'), portas=label,
                 ponte=bridge[0], robo=robot[0] if robot else None,
                 argv=sys.argv, rate_hz=args.rate)
        rec.start_snapshots(lambda: {
            'numbers': numbers_of(st, sender, label),
            'verdict': diagnose(st, robot is not None, True)[1],
        })
        print(f'gravando a sessão em {rec.path}')

    Handler.state = st
    Handler.sender = sender
    Handler.recorder = rec
    Handler.ports_label = label
    Handler.bridge_present = True
    Handler.robot_present = robot is not None

    socketserver.ThreadingTCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(('0.0.0.0', args.http_port), Handler) as srv:
        print(f'painel em http://localhost:{args.http_port}')
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            print('\nparando motores...')
        finally:
            sender.running = False
            for r in readers:
                r.running = False
            for _ in range(5):
                os.write(bridge[1], build_packet(st.robot_id, 0.0, 0.0))
                time.sleep(0.02)
            for _, (fd, _k, _d) in found.items():
                os.close(fd)
            if rec:
                rec.close()
                print(f'log da sessão: {rec.path}')


if __name__ == '__main__':
    main()
