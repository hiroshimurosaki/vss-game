#!/usr/bin/env python3
"""Console web para dirigir os robôs pelo rádio, sem ROS.

    ./tools/radio_console.py
    # abre http://localhost:8060

Fala direto com a ponte pela serial, montando o mesmo pacote de 14 bytes que o
nó `radio_communication` monta. Existe pela mesma razão do `radio_test.py`:
responder "o problema é o robô ou é o ROS?" sem subir o ROS. A diferença é que
aqui dá para dirigir com as duas rodas na mão, ver o pacote saindo e trocar de
robô sem reiniciar nada — que é o que se quer com o robô na bancada.

Só stdlib: nada de pyserial, nada de aiohttp. Roda num clone do repo recém
baixado, sem `colcon build` e sem `source`.

Segurança que não é opcional com robô na mesa:

- **watchdog no navegador**: a página manda o comando a 20 Hz enquanto a tecla
  está pressionada. Se a aba fechar, travar ou a rede cair, o servidor para de
  receber e zera os motores em 300 ms sozinho. Sem isso, um travamento do
  navegador deixa o robô acelerado contra a parede.
- **soltar a tecla para = parar**, e a barra de espaço é freio de emergência.
"""

import argparse
import http.server
import json
import os
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
    # Abrir a porta reseta o Nano (DTR). Sem esta pausa os primeiros pacotes
    # caem no bootloader e somem, e parece que o rádio não está funcionando.
    time.sleep(2.0)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


class Sender(threading.Thread):
    """Manda o comando corrente a taxa fixa, e zera se o navegador sumir.

    Taxa fixa em vez de mandar a cada clique porque o firmware do robô tem
    `COMMAND_TIMEOUT` de 1 s: sem fluxo contínuo ele para sozinho no meio de uma
    manobra. E o watchdog é medido aqui, no lado que segura a serial, porque é o
    único ponto que continua vivo se o navegador morrer.
    """

    def __init__(self, fd, rate_hz=30.0, watchdog_s=0.3):
        super().__init__(daemon=True)
        self.fd = fd
        self.period = 1.0 / rate_hz
        self.watchdog = watchdog_s
        self.lock = threading.Lock()
        self.robot_id = 0
        self.left = 0.0
        self.right = 0.0
        self.last_cmd = 0.0
        self.sent = 0
        self.errors = 0
        self.stale = True
        self.running = True
        # Preenchidos pelo Reader quando a ponte roda o tx_probe.
        self.banner = ''
        self.ack_ok = None
        self.ack_total = None
        self.ack_at = 0.0

    def set(self, robot_id, left, right):
        with self.lock:
            self.robot_id = int(robot_id)
            self.left = float(left)
            self.right = float(right)
            self.last_cmd = time.time()

    def stop_motors(self):
        with self.lock:
            self.left = self.right = 0.0
            self.last_cmd = time.time()

    def snapshot(self):
        with self.lock:
            fresh = (time.time() - self.ack_at) < 2.0
            return {
                'robot_id': self.robot_id,
                'left': round(self.left, 3),
                'right': round(self.right, 3),
                'sent': self.sent,
                'errors': self.errors,
                'stale': self.stale,
                'banner': self.banner,
                'ack_ok': self.ack_ok if fresh else None,
                'ack_total': self.ack_total if fresh else None,
            }

    def run(self):
        while self.running:
            with self.lock:
                age = time.time() - self.last_cmd
                self.stale = age > self.watchdog
                left = 0.0 if self.stale else self.left
                right = 0.0 if self.stale else self.right
                rid = self.robot_id
            try:
                os.write(self.fd, build_packet(rid, left, right))
                self.sent += 1
            except OSError:
                self.errors += 1
            time.sleep(self.period)


class Reader(threading.Thread):
    """Lê o que a ponte devolve pela serial.

    O `tx_probe.ino` responde `PROBE RADIO OK|FAIL` no boot e `ACK ok/total` a
    cada 500 ms. É isso que permite saber se o robô recebeu **sem olhar para o
    robô**: o `ok` vem da confirmação em hardware do nRF24, não de código do
    lado de lá.

    Com o `tx_bridge.ino` ou o `Tx_Arduino.ino` gravados não vem nada, e a
    interface simplesmente mostra "ponte não informa" — ler é inofensivo nos
    dois casos.
    """

    def __init__(self, fd, sender):
        super().__init__(daemon=True)
        self.fd = fd
        self.sender = sender
        self.running = True

    def run(self):
        buf = b''
        while self.running:
            try:
                data = os.read(self.fd, 256)
            except (BlockingIOError, OSError):
                data = b''
            if not data:
                time.sleep(0.05)
                continue

            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._handle(line.decode('utf-8', 'replace').strip())
            if len(buf) > 512:
                buf = buf[-128:]

    def _handle(self, line):
        if not line:
            return
        s = self.sender
        if line.startswith('ACK'):
            try:
                ok, total = line.split()[1].split('/')
                with s.lock:
                    s.ack_ok, s.ack_total = int(ok), int(total)
                    s.ack_at = time.time()
            except (ValueError, IndexError):
                pass
        else:
            with s.lock:
                s.banner = line[:60]


PAGE = """<!DOCTYPE html>
<html lang="pt-BR"><head><meta charset="utf-8">
<title>Console do rádio — VSS</title>
<style>
 :root { color-scheme: dark; }
 * { box-sizing: border-box; }
 body { margin:0; background:#12141a; color:#e6e8ee;
        font:14px/1.55 system-ui,-apple-system,Segoe UI,sans-serif; }
 header { padding:12px 18px; background:#1b1e26; border-bottom:1px solid #2b3040;
          display:flex; gap:16px; align-items:center; flex-wrap:wrap; }
 h1 { font-size:15px; margin:0; font-weight:600; }
 main { padding:18px; display:flex; gap:18px; flex-wrap:wrap; align-items:flex-start; }
 .card { background:#1b1e26; border:1px solid #2b3040; border-radius:8px;
         padding:14px 16px; min-width:290px; }
 .card h2 { font-size:12px; margin:0 0 10px; text-transform:uppercase;
            letter-spacing:.07em; color:#8f97ad; font-weight:600; }
 .pad { display:grid; grid-template-columns:repeat(3,72px);
        grid-template-rows:repeat(3,72px); gap:8px; }
 .pad button { font-size:22px; border-radius:8px; }
 .pad .sp { visibility:hidden; }
 button { background:#2b3040; color:#e6e8ee; border:1px solid #39405400;
          border-radius:6px; padding:9px 14px; font-size:14px; cursor:pointer; }
 button:hover { filter:brightness(1.25); }
 button.act { background:#2f6df6; }
 button.stop { background:#c0392b; font-weight:600; }
 .row { display:flex; align-items:center; gap:10px; margin:9px 0; }
 .row label { width:72px; font-size:12px; color:#8f97ad; }
 input[type=range] { flex:1; }
 output { width:52px; text-align:right; font-variant-numeric:tabular-nums; }
 table { border-collapse:collapse; font-size:13px; width:100%; }
 td { padding:3px 0; } td.k { color:#8f97ad; }
 td.v { text-align:right; font-variant-numeric:tabular-nums; }
 code { background:#0e1015; padding:2px 7px; border-radius:4px;
        font-size:12px; letter-spacing:.05em; }
 .ok { color:#4ec98a; } .warn { color:#e0913a; } .bad { color:#e0614f; }
 .hint { color:#8f97ad; font-size:12px; margin:8px 0 0; }
 select { background:#12141a; color:#e6e8ee; border:1px solid #2b3040;
          border-radius:6px; padding:7px 10px; font-size:14px; }
</style></head><body>
<header>
  <h1>Console do rádio</h1>
  <span>robô <select id="rid"><option>0</option><option>1</option>
    <option>2</option><option>3</option></select></span>
  <span id="link" class="hint">—</span>
  <span style="flex:1"></span>
  <button class="stop" id="estop">PARAR (espaço)</button>
</header>
<main>
  <div class="card">
    <h2>Dirigir</h2>
    <div class="pad">
      <div class="sp"></div><button data-k="w">▲</button><div class="sp"></div>
      <button data-k="a">◀</button><button data-k="s">▼</button><button data-k="d">▶</button>
    </div>
    <p class="hint">WASD ou as setas. <b>Soltar para.</b> Q e E giram no lugar.</p>
    <div class="row">
      <label>força</label>
      <input type="range" id="speed" min="0" max="100" value="50">
      <output id="speedv">0.50</output>
    </div>
  </div>

  <div class="card">
    <h2>Roda a roda</h2>
    <div class="row"><label>esquerda</label>
      <input type="range" id="wl" min="-100" max="100" value="0"><output id="wlv">0.00</output></div>
    <div class="row"><label>direita</label>
      <input type="range" id="wr" min="-100" max="100" value="0"><output id="wrv">0.00</output></div>
    <div class="row">
      <button id="applyw" class="act">Manter</button>
      <button id="zerow">Zerar</button>
    </div>
    <p class="hint">"Manter" segura o valor sem tecla pressionada — para medir
    <code>wheel_speed_max</code> cronometrando um metro.</p>
  </div>

  <div class="card">
    <h2>Saindo agora</h2>
    <table>
      <tr><td class="k">robô</td><td class="v" id="s_id">—</td></tr>
      <tr><td class="k">esquerda</td><td class="v" id="s_l">—</td></tr>
      <tr><td class="k">direita</td><td class="v" id="s_r">—</td></tr>
      <tr><td class="k">pacotes</td><td class="v" id="s_n">—</td></tr>
      <tr><td class="k">erros serial</td><td class="v" id="s_e">—</td></tr>
      <tr><td class="k">estado</td><td class="v" id="s_w">—</td></tr>
    </table>
    <p class="hint">Pacote de 14 bytes:</p>
    <p><code id="s_hex">—</code></p>
  </div>

  <div class="card">
    <h2>O robô recebeu?</h2>
    <div id="ackbig" style="font-size:34px;font-weight:600;margin:4px 0 2px">—</div>
    <div id="acksub" class="hint" style="margin:0">aguardando a ponte</div>
    <table style="margin-top:12px">
      <tr><td class="k">rádio da ponte</td><td class="v" id="s_banner">—</td></tr>
      <tr><td class="k">confirmados</td><td class="v" id="s_ack">—</td></tr>
    </table>
    <p class="hint">Isto vem do <b>auto-ACK do nRF24</b>: o rádio do robô
    confirma cada pacote em hardware. Entrega alta prova que o outro rádio
    recebeu — mesmo que o robô não se mexa, o que aí isola o problema em motor,
    bateria ou firmware.</p>
    <p class="hint">Exige a ponte com <code>firmware/tx_probe</code>. Com o
    <code>tx_bridge</code> ou o <code>Tx_Arduino</code> não há como saber.</p>
  </div>
</main>
<script>
let speed = 0.5, held = new Set(), hold = null;

const KEYS = {
  w:[1,1], arrowup:[1,1], s:[-1,-1], arrowdown:[-1,-1],
  a:[-0.6,0.6], arrowleft:[-0.6,0.6], d:[0.6,-0.6], arrowright:[0.6,-0.6],
  q:[-1,1], e:[1,-1],
};

function wheels() {
  if (hold) return hold;
  let l = 0, r = 0;
  for (const k of held) { const v = KEYS[k]; if (v) { l += v[0]; r += v[1]; } }
  const m = Math.max(1, Math.abs(l), Math.abs(r));
  return [l/m*speed, r/m*speed];
}

async function tick() {
  const [l, r] = wheels();
  try {
    const res = await fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({id: +document.getElementById('rid').value, left: l, right: r})});
    const s = await res.json();
    document.getElementById('s_id').textContent = s.robot_id;
    document.getElementById('s_l').textContent = s.left.toFixed(2);
    document.getElementById('s_r').textContent = s.right.toFixed(2);
    document.getElementById('s_n').textContent = s.sent;
    document.getElementById('s_e').textContent = s.errors;
    const w = document.getElementById('s_w');
    w.textContent = s.stale ? 'parado (watchdog)' : 'enviando';
    w.className = 'v ' + (s.stale ? 'warn' : 'ok');
    document.getElementById('s_hex').textContent = s.hex;
    document.getElementById('link').textContent = s.port;

    const big = document.getElementById('ackbig'), sub = document.getElementById('acksub');
    document.getElementById('s_banner').textContent = s.banner || '—';
    if (s.ack_total === null || s.ack_total === undefined) {
      big.textContent = '—'; big.className = '';
      sub.textContent = s.banner ? 'ponte não reporta entrega' : 'ponte não informa (grave o tx_probe)';
      document.getElementById('s_ack').textContent = '—';
    } else if (s.ack_total === 0) {
      big.textContent = '—'; big.className = '';
      sub.textContent = 'nenhum pacote enviado nesta janela';
      document.getElementById('s_ack').textContent = '0/0';
    } else {
      const pct = 100 * s.ack_ok / s.ack_total;
      big.textContent = pct.toFixed(0) + '%';
      big.className = pct > 90 ? 'ok' : (pct > 0 ? 'warn' : 'bad');
      sub.textContent = pct > 90 ? 'o robô está recebendo'
                      : pct > 0  ? 'link instável — distância, antena ou alimentação'
                                 : 'nada chega ao robô';
      document.getElementById('s_ack').textContent = s.ack_ok + '/' + s.ack_total;
    }
  } catch (e) {
    document.getElementById('s_w').textContent = 'sem servidor';
  }
}
setInterval(tick, 50);

addEventListener('keydown', e => {
  const k = e.key.toLowerCase();
  if (k === ' ') { held.clear(); hold = null; e.preventDefault(); return; }
  if (KEYS[k]) { held.add(k); hold = null; e.preventDefault(); }
});
addEventListener('keyup', e => held.delete(e.key.toLowerCase()));
// Perder o foco da janela com a tecla presa deixaria o robô acelerado: o
// keyup nunca chega. Limpar aqui é o que evita o robô fugindo pela mesa.
addEventListener('blur', () => { held.clear(); hold = null; });

for (const b of document.querySelectorAll('.pad button')) {
  const k = b.dataset.k;
  const on = e => { e.preventDefault(); held.add(k); hold = null; b.classList.add('act'); };
  const off = () => { held.delete(k); b.classList.remove('act'); };
  b.addEventListener('mousedown', on); b.addEventListener('touchstart', on);
  addEventListener('mouseup', off); b.addEventListener('touchend', off);
  b.addEventListener('mouseleave', off);
}

const sp = document.getElementById('speed');
sp.oninput = () => { speed = sp.value/100; document.getElementById('speedv').textContent = speed.toFixed(2); };
sp.oninput();

const wl = document.getElementById('wl'), wr = document.getElementById('wr');
const showw = () => { document.getElementById('wlv').textContent=(wl.value/100).toFixed(2);
                      document.getElementById('wrv').textContent=(wr.value/100).toFixed(2); };
wl.oninput = wr.oninput = showw; showw();
document.getElementById('applyw').onclick = () => { held.clear(); hold=[wl.value/100, wr.value/100]; };
document.getElementById('zerow').onclick  = () => { wl.value=wr.value=0; showw(); hold=null; };
document.getElementById('estop').onclick  = () => { held.clear(); hold=null; };
</script></body></html>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    sender = None
    port_name = ''

    def log_message(self, *a):
        pass                      # o console manda 20 req/s; log encheria a tela

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
        if self.path != '/cmd':
            self.send_error(404)
            return
        n = int(self.headers.get('Content-Length', 0))
        try:
            data = json.loads(self.rfile.read(n) or b'{}')
        except json.JSONDecodeError:
            data = {}

        s = self.sender
        s.set(data.get('id', 0), data.get('left', 0.0), data.get('right', 0.0))

        snap = s.snapshot()
        pkt = build_packet(snap['robot_id'],
                           0.0 if snap['stale'] else snap['left'],
                           0.0 if snap['stale'] else snap['right'])
        snap['hex'] = ' '.join(f'{b:02x}' for b in pkt)
        snap['port'] = self.port_name
        self._json(snap)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # O caminho by-id é estável: só a ponte tem número de série, então ele não
    # troca de lugar quando outro Arduino entra na USB. Usar ttyUSB0 direto já
    # mandou pacote para o Arduino errado.
    ap.add_argument('--port',
                    default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--http-port', type=int, default=8060)
    ap.add_argument('--rate', type=float, default=30.0)
    args = ap.parse_args()

    port = args.port
    if not os.path.exists(port):
        # ttyACM na lista porque a ponte pode ser um Uno oficial, que enumera
        # pelo ATmega16U2 e nunca aparece como ttyUSB. Isto é chute pelo nome do
        # device, não identificação: se houver mais de uma placa plugada, passe
        # --port $(./tools/porta.sh), que decide pelo banner de boot.
        candidatos = [c for c in ('/dev/ttyUSB0', '/dev/ttyACM0')
                      if os.path.exists(c)]
        if candidatos:
            print(f'{port} não existe; usando {candidatos[0]}', file=sys.stderr)
            port = candidatos[0]
        else:
            sys.exit(f'sem serial: nem {port} nem {candidatos} existem.\n'
                     'A ponte está ligada? No Pop!_OS o brltty pode roubar o '
                     'CH340 — ver CLAUDE.md.')

    print(f'abrindo {port} a {args.baud} (o Nano reseta ao abrir, aguardando 2s)')
    fd = open_serial(port, args.baud)

    sender = Sender(fd, rate_hz=args.rate)
    sender.start()

    reader = Reader(fd, sender)
    reader.start()

    Handler.sender = sender
    Handler.port_name = os.path.basename(port)

    socketserver.ThreadingTCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(('0.0.0.0', args.http_port), Handler) as srv:
        print(f'console em http://localhost:{args.http_port}')
        print('mandando 14 bytes a %.0f Hz. Ctrl-C para sair.' % args.rate)
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            print('\nparando motores...')
        finally:
            sender.running = False
            reader.running = False
            for _ in range(5):
                os.write(fd, build_packet(sender.robot_id, 0.0, 0.0))
                time.sleep(0.02)
            os.close(fd)


if __name__ == '__main__':
    main()
