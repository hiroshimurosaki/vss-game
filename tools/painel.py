#!/usr/bin/env python3
"""Painel único: junta a cadeia da bancada e a cadeia do ROS numa tela só.

    ./tools/painel.py
    # sobe o que falta e abre http://localhost:8062
    # Ctrl-C derruba tudo que ELE subiu, e só isso

Um comando. Ele sobe os dois back-ends, reaproveita o que já estiver de pé e
serve a tela. Sem `source`, sem três terminais, sem decorar três portas.

Existe porque o diagnóstico deste projeto estava repartido em duas telas que
respondem a mesma pergunta — "onde a informação morre?" — em dois trechos
diferentes da cadeia, e você precisava saber de antemão qual das duas abrir:

    tools/debug_panel.py   :8061   serial → rádio → robô → PWM   (stack PARADO)
    ros2 run diagnostics flow_panel  :8050   câmera → /motorVelocities  (stack RODANDO)

As duas são excludentes por um motivo físico, não por descuido: serial não abre
duas vezes. Com o jogo de pé quem tem a serial da ponte é o `radio_communication`;
na bancada quem tem é o `debug_panel`. Este arquivo não resolve isso e nem tenta
— ele **pergunta às duas** e mostra a que estiver viva, dizendo na tela por que
a outra não está. Às 7h da manhã da feira, é um endereço só para decorar.

O QUE ELE NÃO É
    Não é um terceiro motor de diagnóstico. Toda a inteligência continua nos
    dois painéis originais; aqui só há transporte e tradução. Nenhuma linha dos
    dois back-ends foi tocada — se este arquivo sumir, nada quebra.

POR QUE ELE PRECISA MANDAR COMANDO, E NÃO SÓ LER
    O `debug_panel` não tem endpoint de leitura: o estado dele volta como
    resposta ao `POST /cmd`. E esse POST registra quem chamou como cliente —
    dois clientes ao mesmo tempo é um dos vermelhos dele ("os comandos se
    alternam e a medição não vale"). Um observador passivo dispararia esse
    alarme só de observar. Então este painel assume o posto de cliente único e
    repassa o teclado. Corolário: **não deixe esta aba e o :8061 abertos ao
    mesmo tempo** — o alarme de dois clientes vai acender, e estará certo.

O QUE ELE NUNCA MATA
    Só derruba processo que ele mesmo subiu. Um `debug_panel` que já estava no
    ar é reaproveitado e continua vivo no Ctrl-C — porque pode ser a janela de
    outra pessoa, e diagnóstico que mata o trabalho alheio some da bancada.
    A serial presa por outro processo ele **reporta**, não resolve: liberar
    exige `--matar`, dito na mão.

Só stdlib: nada de pyserial, nada de aiohttp, nada de rclpy. Mesmo motivo do
`debug_panel` — precisa subir quando o workspace não compila. O lado ROS é a
única exceção, e por isso ele é opcional: se o workspace não estiver compilado,
a coluna do ROS fica offline e o resto funciona igual.
"""

import argparse
import http.server
import json
import os
import signal
import socketserver
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)

#: Vocabulário de status. Os dois painéis já usavam exatamente estes quatro
#: valores, o que é a razão de a fusão ser tradução e não reescrita.
OK, BAD, WARN, UNKNOWN = 'ok', 'bad', 'warn', 'unknown'

#: O lado ROS muda devagar (nós subindo, tópicos parando) e custa uma volta em
#: aiohttp. Não faz sentido pedir a 10 Hz junto com o comando; meio segundo é
#: imediato para quem está depurando de pé e não afoga o nó.
FLOW_TTL = 0.5

#: Curto de propósito: este timeout entra no caminho do comando do teclado. Um
#: back-end morto não pode segurar o laço de 10 Hz, senão o painel engasga
#: justamente quando você mais precisa dele.
TIMEOUT = 0.4


def _post(url: str, payload: dict, timeout: float = TIMEOUT):
    body = json.dumps(payload).encode()
    req = urllib.request.Request(url, data=body, method='POST',
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read() or b'{}')


def _get(url: str, timeout: float = TIMEOUT):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read() or b'{}')


# ── Tradução ─────────────────────────────────────────────────────────────────
# O `debug_panel` fala inglês nas chaves e o `flow_panel` fala português. O
# repo é português, então o inglês é que se move. Traduzir aqui, e não lá,
# mantém a promessa de não tocar nos originais.

def _elo_bancada(link: dict) -> dict:
    return {
        'nome': link.get('name', '—'),
        'status': link.get('status', UNKNOWN),
        'detalhe': link.get('detail', ''),
        'dica': link.get('hint', ''),
    }


def _veredito_bancada(v: dict) -> dict:
    return {
        'nivel': v.get('level', UNKNOWN),
        'onde': v.get('where', '—'),
        'texto': v.get('text', ''),
        'dica': v.get('hint', ''),
    }


def _elo_ros(elo: dict) -> dict:
    """Achata o elo do ROS no mesmo formato de quatro campos da bancada.

    O `hz` entra no detalhe porque é o número que decide se o elo está vivo ou
    arrastando, e sem ele o texto do `flow_panel` perde a metade quantitativa.
    """
    detalhe = elo.get('detalhe', '')
    hz, nominal = elo.get('hz'), elo.get('hz_nominal')
    if hz is not None and nominal:
        detalhe = f'{detalhe} — {hz:g} Hz de {nominal:g}'.strip(' —')
    return {
        'nome': elo.get('topico', '—'),
        'status': elo.get('status', UNKNOWN),
        'detalhe': detalhe,
        'dica': elo.get('dica', '') or elo.get('normal_se', ''),
    }


# ── Coleta ───────────────────────────────────────────────────────────────────

class Fontes:
    """Fala com os dois back-ends e guarda o último estado bom de cada um."""

    def __init__(self, url_bancada: str, url_ros: str):
        self.url_bancada = url_bancada.rstrip('/')
        self.url_ros = url_ros.rstrip('/')
        self.lock = threading.Lock()
        self.ros = {'online': False, 'erro': 'ainda não consultado'}
        self._ros_em = 0.0

    # O lado ROS roda num thread próprio: assim o comando do teclado nunca
    # espera por ele, mesmo que o nó esteja subindo ou morrendo.
    def girar_ros(self):
        while True:
            try:
                d = _get(f'{self.url_ros}/estado')
                novo = {
                    'online': True,
                    'elos': [_elo_ros(e) for e in d.get('elos', [])],
                    'nos': d.get('nos', []),
                    'veredito': d.get('veredito', {}),
                    'estado_jogo': d.get('estado_jogo', '—'),
                    'uptime': d.get('uptime'),
                }
            except (urllib.error.URLError, OSError, ValueError) as exc:
                novo = {'online': False, 'erro': _motivo(exc, 'flow_panel',
                        'ros2 run diagnostics flow_panel')}
            with self.lock:
                self.ros = novo
                self._ros_em = time.time()
            time.sleep(FLOW_TTL)

    def bancada(self, cid: str, robot_id: int, left: float, right: float) -> dict:
        try:
            d = _post(f'{self.url_bancada}/cmd',
                      {'cid': cid, 'id': robot_id, 'left': left, 'right': right})
        except (urllib.error.URLError, OSError, ValueError) as exc:
            return {'online': False, 'erro': _motivo(exc, 'debug_panel',
                    './tools/debug_panel.py')}
        return {
            'online': True,
            'elos': [_elo_bancada(l) for l in d.get('links', [])],
            'veredito': _veredito_bancada(d.get('verdict', {})),
            'numeros': d.get('numbers', {}),
        }

    def instantaneo_ros(self) -> dict:
        with self.lock:
            return dict(self.ros)


def _motivo(exc, quem: str, comando: str) -> str:
    """Mensagem de offline que diz o que fazer, não só que falhou."""
    if isinstance(exc, urllib.error.URLError) and 'refused' in str(exc.reason).lower():
        return f'o {quem} não está no ar — suba com `{comando}`'
    return f'{quem}: {exc}'


def veredito_geral(bancada: dict, ros: dict) -> dict:
    """Uma frase no topo. Regra: quem está vermelho fala primeiro.

    Entre os dois, o ROS vem antes porque é o trecho de cima da cadeia: com a
    visão morta, a bancada nem chega a ser consultada de verdade. É a mesma
    lógica de ordem que cada painel já aplica dentro de si.
    """
    vivos = [x for x in (ros, bancada) if x.get('online')]
    if not vivos:
        return {'nivel': UNKNOWN, 'onde': 'nenhum back-end no ar',
                'texto': 'nem o painel da bancada nem o do ROS respondem.',
                'dica': 'Na bancada: ./tools/debug_panel.py. Com o jogo de pé: '
                        'ros2 run diagnostics flow_panel.'}
    for lado, rotulo in ((ros, 'ROS'), (bancada, 'bancada')):
        v = lado.get('veredito') or {}
        if lado.get('online') and v.get('nivel') == BAD:
            return dict(v, onde=f'{rotulo} · {v.get("onde", "—")}')
    for lado, rotulo in ((ros, 'ROS'), (bancada, 'bancada')):
        v = lado.get('veredito') or {}
        if lado.get('online') and v.get('nivel') == WARN:
            return dict(v, onde=f'{rotulo} · {v.get("onde", "—")}')
    v = (vivos[0].get('veredito') or {})
    return dict(v, onde=v.get('onde', '—'))


PAGE = r"""<!doctype html>
<html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Painel único — VSS</title>
<style>
 :root { --bg:#0f1220; --card:#171b2e; --line:#39405f; --txt:#e8ebf5;
         --dim:#98a0bd; --ok:#4ec98a; --bad:#e0614f; --warn:#e0913a;
         --unknown:#5b6480; }
 * { box-sizing:border-box; }
 body { margin:0; background:var(--bg); color:var(--txt); font:15px/1.5
        system-ui,-apple-system,Segoe UI,Roboto,sans-serif; padding:18px; }
 h1 { font-size:19px; margin:0 0 4px; }
 .sub { color:var(--dim); font-size:13px; margin-bottom:16px; }
 .verdict { border-left:4px solid var(--unknown); background:var(--card);
            padding:12px 16px; border-radius:6px; margin-bottom:16px; }
 .verdict.ok{border-color:var(--ok)} .verdict.bad{border-color:var(--bad)}
 .verdict.warn{border-color:var(--warn)} .verdict.unknown{border-color:var(--unknown)}
 .verdict .big { font-size:17px; font-weight:600; }
 .verdict .hint { color:var(--dim); font-size:13px; margin-top:4px; }
 .cols { display:grid; grid-template-columns:repeat(auto-fit,minmax(340px,1fr));
         gap:14px; }
 .card { background:var(--card); border-radius:8px; padding:14px 16px; }
 .card h2 { font-size:14px; margin:0 0 2px; text-transform:uppercase;
            letter-spacing:.06em; }
 .card .when { color:var(--dim); font-size:12px; margin-bottom:10px; }
 .off { color:var(--dim); font-size:13px; padding:10px 0; }
 .elo { display:flex; gap:10px; padding:7px 0; border-top:1px solid #232842; }
 .elo:first-of-type { border-top:0; }
 .dot { width:9px; height:9px; border-radius:50%; margin-top:7px; flex:none; }
 .dot.ok{background:var(--ok)} .dot.bad{background:var(--bad)}
 .dot.warn{background:var(--warn)} .dot.unknown{background:var(--unknown)}
 .elo .n { font-weight:600; font-size:14px; }
 .elo .d { color:var(--dim); font-size:13px; }
 .elo .h { color:#7f88a8; font-size:12px; font-style:italic; margin-top:2px; }
 .nums { display:flex; flex-wrap:wrap; gap:14px; margin-top:10px;
         border-top:1px solid #232842; padding-top:10px; font-size:13px; }
 .nums b { color:var(--dim); font-weight:400; }
 .ctrl { background:var(--card); border-radius:8px; padding:14px 16px;
         margin-top:14px; display:flex; gap:18px; align-items:center;
         flex-wrap:wrap; }
 button { background:#232842; color:var(--txt); border:1px solid var(--line);
          border-radius:6px; padding:7px 14px; font-size:14px; cursor:pointer; }
 button.stop { border-color:var(--bad); color:#ffb3a8; }
 kbd { background:#232842; border:1px solid var(--line); border-radius:4px;
       padding:1px 6px; font-size:12px; }
</style></head><body>
<h1>Painel único</h1>
<div class="sub">Uma tela para as duas cadeias. A inteligência continua no
  <code>debug_panel.py</code> (:8061) e no <code>flow_panel</code> (:8050) —
  aqui só há transporte e tradução.</div>

<div class="verdict unknown" id="v">
  <div class="big"><span id="v_onde">—</span>: <span id="v_txt">conectando…</span></div>
  <div class="hint" id="v_dica"></div>
</div>

<div class="cols">
  <div class="card">
    <h2>Bancada</h2>
    <div class="when">serial → rádio → robô → PWM · exige o stack parado</div>
    <div id="b_off" class="off"></div>
    <div id="b_elos"></div>
    <div class="nums" id="b_nums"></div>
  </div>
  <div class="card">
    <h2>ROS</h2>
    <div class="when">câmera → <code>/motorVelocities</code> · exige o jogo rodando</div>
    <div id="r_off" class="off"></div>
    <div id="r_elos"></div>
    <div class="nums" id="r_nums"></div>
  </div>
</div>

<div class="ctrl">
  <span><kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> anda ·
        <kbd>Q</kbd><kbd>E</kbd> gira no lugar · <kbd>espaço</kbd> para</span>
  <span>robô <select id="rid"><option>0</option><option selected>1</option></select></span>
  <span>força <input type="range" id="sp" min="0" max="100" value="60">
        <output id="spv">0.60</output></span>
  <button id="boost">Pulso de 1 s</button>
  <button class="stop" id="estop">PARAR</button>
</div>

<script>
// Identidade estável desta aba. O debug_panel distingue "duas abas disputando"
// de "um cliente reconectando" por este campo.
const CID = 'painel-' + Math.random().toString(36).slice(2, 8);
// Mesmo mapa de teclas do debug_panel, de propósito: quem já usa aquele não
// reaprende nada.
const KEYS = { w:[1,1], arrowup:[1,1], s:[-1,-1], arrowdown:[-1,-1],
  a:[-0.6,0.6], arrowleft:[-0.6,0.6], d:[0.6,-0.6], arrowright:[0.6,-0.6],
  q:[-1,1], e:[1,-1] };
let speed = 0.6, held = new Set();

function wheels() {
  let l = 0, r = 0;
  for (const k of held) { const v = KEYS[k]; if (v) { l += v[0]; r += v[1]; } }
  const m = Math.max(1, Math.abs(l), Math.abs(r));
  return [l/m*speed, r/m*speed];
}

function elos(alvo, lista) {
  alvo.innerHTML = (lista || []).map(e => `
    <div class="elo"><div class="dot ${e.status}"></div><div>
      <div class="n">${e.nome}</div>
      <div class="d">${e.detalhe || ''}</div>
      ${e.dica ? `<div class="h">${e.dica}</div>` : ''}
    </div></div>`).join('');
}

function nums(alvo, pares) {
  alvo.style.display = pares.length ? 'flex' : 'none';
  alvo.innerHTML = pares.map(([k, v]) => `<span><b>${k}</b> ${v}</span>`).join('');
}

function lado(pref, d, extras) {
  const off = document.getElementById(pref + '_off');
  off.textContent = d.online ? '' : (d.erro || 'fora do ar');
  off.style.display = d.online ? 'none' : 'block';
  elos(document.getElementById(pref + '_elos'), d.online ? d.elos : []);
  nums(document.getElementById(pref + '_nums'), d.online ? extras(d) : []);
}

function render(s) {
  const v = document.getElementById('v');
  v.className = 'verdict ' + (s.geral.nivel || 'unknown');
  document.getElementById('v_onde').textContent = s.geral.onde || '—';
  document.getElementById('v_txt').textContent = s.geral.texto || '';
  document.getElementById('v_dica').textContent = s.geral.dica || '';

  lado('b', s.bancada, d => {
    const n = d.numeros || {};
    const out = [];
    if (n.fw_id !== undefined && n.fw_id !== null) out.push(['ID no firmware', n.fw_id]);
    if (n.tx !== undefined) out.push(['serial', (n.tx || 0).toFixed(0) + '/s']);
    if (n.rx_ok !== undefined) out.push(['aceitos', (n.rx_ok || 0).toFixed(0) + '/s']);
    if (n.rx_alheio) out.push(['descartados por ID', (n.rx_alheio).toFixed(0) + '/s']);
    if (n.pwm_a !== null && n.pwm_a !== undefined) out.push(['PWM', n.pwm_a + ' / ' + n.pwm_b]);
    return out;
  });
  lado('r', s.ros, d => {
    const out = [];
    if (d.estado_jogo) out.push(['estado do jogo', d.estado_jogo]);
    const mortos = (d.nos || []).filter(n => n.status === 'bad').map(n => n.rotulo);
    if (mortos.length) out.push(['nós ausentes', mortos.join(', ')]);
    return out;
  });
}

async function tick() {
  const [l, r] = wheels();
  try {
    const resp = await fetch('/estado', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({cid: CID, id: +document.getElementById('rid').value,
                            left: l, right: r})
    });
    render(await resp.json());
  } catch (_) { /* o próprio painel caiu; o próximo tick tenta de novo */ }
}
setInterval(tick, 100);
tick();

addEventListener('keydown', e => {
  const k = e.key.toLowerCase();
  if (k === ' ') { held.clear(); e.preventDefault(); return; }
  if (KEYS[k]) { held.add(k); e.preventDefault(); }
});
addEventListener('keyup', e => held.delete(e.key.toLowerCase()));
// Perder o foco com a tecla presa deixaria o robô acelerado: o keyup nunca chega.
addEventListener('blur', () => held.clear());

const sp = document.getElementById('sp');
sp.oninput = () => { speed = sp.value/100;
  document.getElementById('spv').textContent = speed.toFixed(2); };
document.getElementById('estop').onclick = () => held.clear();
document.getElementById('boost').onclick = () => fetch('/boost', {method:'POST'});
</script></body></html>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    fontes: Fontes = None

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
        if self.path == '/boost':
            try:
                _post(f'{self.fontes.url_bancada}/boost', {})
            except (urllib.error.URLError, OSError, ValueError):
                pass              # sem bancada não há pulso; o painel já diz isso
            self._json({'ok': True})
            return
        if self.path != '/estado':
            self.send_error(404)
            return

        n = int(self.headers.get('Content-Length', 0))
        try:
            d = json.loads(self.rfile.read(n) or b'{}')
        except json.JSONDecodeError:
            d = {}

        bancada = self.fontes.bancada(
            str(d.get('cid', self.client_address[0])),
            int(d.get('id', 1)), float(d.get('left', 0.0)),
            float(d.get('right', 0.0)))
        ros = self.fontes.instantaneo_ros()
        self._json({'bancada': bancada, 'ros': ros,
                    'geral': veredito_geral(bancada, ros)})


# ── Orquestração ─────────────────────────────────────────────────────────────
# A parte que transforma três comandos em um. Regra única e não negociável:
# subir só o que falta, matar só o que subiu.

def no_ar(url: str) -> bool:
    """O back-end já responde? Barato, e é o que torna o comando idempotente."""
    try:
        urllib.request.urlopen(url.rstrip('/') + '/', timeout=0.4).read(1)
        return True
    except (urllib.error.URLError, OSError, ValueError):
        return False


def _porta_de(url: str, padrao: int) -> int:
    try:
        return int(url.rstrip('/').rsplit(':', 1)[1])
    except (IndexError, ValueError):
        return padrao


def quem_segura_seriais():
    """[(pid, comando, porta)] de quem está com um /dev/ttyUSB* aberto.

    O `debug_panel` precisa das duas seriais em exclusivo. O erro clássico é
    subir com o `radio_console` ainda de pé: a porta não abre, o painel fica
    meio cego e a mensagem que aparece é sobre a serial, não sobre o culpado.
    Descobrir o culpado é o que essa função existe para fazer.
    """
    presos = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            for fd in os.listdir(f'/proc/{pid}/fd'):
                alvo = os.readlink(f'/proc/{pid}/fd/{fd}')
                if alvo.startswith('/dev/ttyUSB'):
                    with open(f'/proc/{pid}/cmdline', 'rb') as f:
                        cmd = f.read().replace(b'\0', b' ').decode().strip()
                    presos.append((int(pid), cmd or '?', alvo))
                    break
        except (OSError, PermissionError):
            continue              # processo de outro usuário, ou já morreu
    return presos


def _spawn(argv, rotulo: str, log_path: str):
    log = open(log_path, 'wb')
    # start_new_session: o filho sai do grupo de processos do terminal, senão o
    # Ctrl-C chega nele por conta própria e a mensagem de encerramento fica
    # embaralhada com o traceback dele.
    p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                         cwd=RAIZ, start_new_session=True)
    print(f'  subindo {rotulo} (pid {p.pid}, log em {log_path})')
    return p


def subir(args, tmp: str):
    """Sobe o que falta.

    Devolve [(processo, url, rótulo)] — só do que ELE subiu. O encerramento e a
    espera usam essa lista, e é o que garante que nada alheio seja tocado.
    """
    filhos = []

    if no_ar(args.bancada):
        print(f'  bancada  já no ar em {args.bancada} — reaproveitando')
    else:
        presos = quem_segura_seriais()
        if presos and not args.matar:
            print('  bancada  NÃO vou subir: a serial está presa por')
            for pid, cmd, dev in presos:
                print(f'             pid {pid} em {dev} — {cmd[:60]}')
            print('             rode de novo com --matar, ou feche na mão.')
        else:
            for pid, cmd, dev in presos:
                print(f'  liberando {dev}: matando pid {pid} ({cmd[:40]})')
                try:
                    os.kill(pid, signal.SIGTERM)
                except OSError:
                    pass
            if presos:
                time.sleep(0.6)
            # A porta vem da URL, não do default do debug_panel: sem isto, um
            # `--bancada` apontando para outra porta subiria o back-end na 8061
            # e o painel ficaria falando com o vazio.
            porta = _porta_de(args.bancada, 8061)
            filhos.append((_spawn([sys.executable, os.path.join(AQUI, 'debug_panel.py'),
                                   '--http-port', str(porta)],
                                  f'debug_panel :{porta}', f'{tmp}/debug_panel.log'),
                           args.bancada, 'bancada'))

    if args.sem_ros:
        print('  ros      pulado (--sem-ros)')
    elif no_ar(args.ros):
        print(f'  ros      já no ar em {args.ros} — reaproveitando')
    elif not os.path.exists(os.path.join(RAIZ, 'install', 'setup.bash')):
        print('  ros      workspace não compilado (sem install/setup.bash) — '
              'a coluna do ROS fica offline')
    else:
        # O único ponto que precisa de bash: `source` não existe fora dele, e
        # o ambiente do ROS 2 é montado por script, não por variável solta.
        cmd = ('source /opt/ros/humble/setup.bash && source install/setup.bash && '
               'exec ros2 run diagnostics flow_panel')
        filhos.append((_spawn(['bash', '-c', cmd], 'flow_panel :8050',
                              f'{tmp}/flow_panel.log'), args.ros, 'ros'))

    return filhos


#: Generoso de propósito. O `debug_panel` descobre quem é a ponte e quem é o
#: robô resetando cada /dev/ttyUSB* e escutando o banner de boot — com duas
#: placas isso passa de 6 s. Um prazo curto imprimia "não respondeu" enquanto o
#: back-end estava subindo normalmente: aviso falso, e dos que fazem desistir.
PRAZO_SUBIDA = 20.0


def esperar(url: str, rotulo: str, prazo: float = PRAZO_SUBIDA):
    fim = time.time() + prazo
    while time.time() < fim:
        if no_ar(url):
            print(f'  {rotulo} respondeu')
            return True
        time.sleep(0.25)
    print(f'  {rotulo} não respondeu em {prazo:g}s — veja o log; o painel sobe assim mesmo')
    return False


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bancada', default='http://localhost:8061',
                    help='URL do tools/debug_panel.py')
    ap.add_argument('--ros', default='http://localhost:8050',
                    help='URL do flow_panel do pacote diagnostics')
    ap.add_argument('--http-port', type=int, default=8062)
    ap.add_argument('--so-tela', action='store_true',
                    help='não sobe nada, só serve a tela (comportamento de cliente puro)')
    ap.add_argument('--sem-ros', action='store_true',
                    help='não tenta subir o flow_panel')
    ap.add_argument('--matar', action='store_true',
                    help='libera as seriais matando quem as segura (radio_console etc)')
    ap.add_argument('--logs', default='/tmp/vss-painel',
                    help='onde jogar a saída dos back-ends')
    args = ap.parse_args()

    # Sem isto o stdout vira buffer de bloco quando a saída não é um terminal, e
    # quem roda com `| tee` ou `nohup` não vê uma linha do que subiu — que é
    # justamente o que este comando tem de útil.
    sys.stdout.reconfigure(line_buffering=True)

    filhos = []
    if not args.so_tela:
        os.makedirs(args.logs, exist_ok=True)
        print('back-ends:')
        filhos = subir(args, args.logs)
        for _, url, rotulo in filhos:
            esperar(url, rotulo)

    fontes = Fontes(args.bancada, args.ros)
    threading.Thread(target=fontes.girar_ros, daemon=True).start()
    Handler.fontes = fontes

    def encerrar():
        # Só os filhos. O que já estava de pé continua de pé — ver a docstring.
        #
        # Mata o GRUPO, não o processo: `ros2 run` bifurca e o nó real fica num
        # neto. Um `terminate()` no invólucro derruba o pai e deixa o neto vivo
        # segurando a :8050 — medido, o órfão sobreviveu ao Ctrl-C. Como cada
        # filho nasce com `start_new_session`, ele é líder do próprio grupo e
        # `killpg` alcança a descendência inteira sem risco de pegar quem não é
        # nosso.
        for p, _, rotulo in filhos:
            if p.poll() is None:
                print(f'  derrubando {rotulo} (pid {p.pid})')
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except OSError:
                    p.terminate()
        for p, _, _ in filhos:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except OSError:
                    p.kill()

    socketserver.ThreadingTCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(('0.0.0.0', args.http_port), Handler) as srv:
        print()
        print(f'  >>>  http://localhost:{args.http_port}  <<<')
        print(f'  bancada -> {args.bancada}   ros -> {args.ros}')
        if filhos:
            print(f'  Ctrl-C derruba os {len(filhos)} back-end(s) que eu subi.')
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            print('\nencerrando…')
        finally:
            encerrar()


if __name__ == '__main__':
    main()
