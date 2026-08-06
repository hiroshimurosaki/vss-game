"""Visão do campo: câmera → /game_data.

Ocupa exatamente a fronteira que o simulador ocupava:

    produz  /game_data     (o mesmo contrato que o sim_node publicava)

Por isso entra no `game.py` no lugar do `simulator` sem que a IA, o
`game_master` ou a TV saibam da troca.

    ros2 run vision_game vision_node

A calibração fica em http://localhost:8070 — clicar os 4 cantos do campo e
ajustar as faixas de cor. Salva em ~/.vss-game/vision.json.

IMPORTANTE: rode com PYTHONNOUSERSITE=1 se houver um numpy 2.x em ~/.local.
O cv2 do apt é compilado contra numpy 1.x e quebra com o numpy do user-site.
Os launch files já cuidam disso.
"""

import asyncio
import json
import math
import os
import subprocess
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from shared_interfaces.msg import AiDebug, GameData, RobotState

from aiohttp import web, WSMsgType
from ament_index_python.packages import get_package_share_directory

from . import config as cfgmod
from .detector import Detector, pick_color, relocate_corners


class _FFmpegCapture:
    """Captura via pipe do ffmpeg, em vez do `cv2.VideoCapture`.

    Motivo, medido nesta câmera a 1920×1080 MJPG:

        cv2.VideoCapture   16,7 fps
        ffmpeg -> pipe     30,1 fps

    O gargalo não é a câmera nem a detecção — é o decode do MJPEG, que o
    OpenCV faz num thread só. O ffmpeg decodifica multithread. Sem isto a
    escolha seria entre resolução e taxa; com isto dá para ter as duas.

    Não acumula latência porque o consumidor é mais rápido que a câmera
    (detecção ~18 ms contra 33 ms de período). Se isso deixar de valer, o
    buffer do pipe vira atraso crescente — é o primeiro lugar para olhar se a
    posição começar a chegar velha.
    """

    def __init__(self, device, width, height, fps):
        self.width, self.height = width, height
        self._nbytes = width * height * 3
        self._buf = bytearray(self._nbytes)
        self._proc = subprocess.Popen(
            ['ffmpeg', '-hide_banner', '-loglevel', 'error',
             '-f', 'v4l2', '-input_format', 'mjpeg',
             '-video_size', f'{width}x{height}', '-framerate', str(fps),
             '-i', device, '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            bufsize=self._nbytes)

    def read(self):
        view = memoryview(self._buf)
        got = 0
        while got < self._nbytes:
            n = self._proc.stdout.readinto(view[got:])
            if not n:
                return False, None
            got += n
        frame = np.frombuffer(self._buf, np.uint8).reshape(
            self.height, self.width, 3)
        return True, frame

    def release(self):
        if self._proc.poll() is None:
            self._proc.kill()
            self._proc.wait(timeout=2)


class _Track:
    """Diferença finita com EMA, por objeto.

    O `GameData` exige velocidade e a câmera só dá posição. Derivar posição
    ruidosa amplifica o ruído, então o filtro não é opcional.

    Deliberadamente não é Kalman: a IA já tolera 100–400 ms de atraso por
    projeto (`reaction_delay`), então gastar latência para ganhar suavidade é
    trocar na direção errada. EMA de primeira ordem é o suficiente e custa uma
    multiplicação.
    """

    def __init__(self, alpha=0.35, angular=False):
        self.alpha = alpha
        self.angular = angular
        self._last = None
        self._t = None
        self.v = 0.0

    def update(self, value, t):
        if self._last is None or self._t is None:
            self._last, self._t = value, t
            return 0.0

        dt = t - self._t
        if dt <= 1e-6:
            return self.v

        d = value - self._last
        if self.angular:
            # Sem isto, cruzar ±π vira um pico de ~2π/dt — na prática o robô
            # "gira 6 rad/s" por um frame e a IA reage a um fantasma.
            d = math.atan2(math.sin(d), math.cos(d))

        raw = d / dt
        self.v += self.alpha * (raw - self.v)
        self._last, self._t = value, t
        return self.v

    def reset(self):
        self._last = self._t = None
        self.v = 0.0


class VisionNode(Node):

    #: Segundos sem nenhum frame bom antes de desistir da câmera. É tempo, não
    #: contagem de frames: com o backoff entre tentativas, "300 frames
    #: perdidos" pode levar dois minutos e meio, e nesse meio tempo o estande
    #: fica parado sem ninguém entender por quê.
    MAX_BLIND_SECONDS = 10.0

    def __init__(self):
        super().__init__('vision_game')

        self.declare_parameter('port', 8070)
        self.declare_parameter('device', '')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('vel_alpha', 0.35)
        self.declare_parameter('publish_overlay', True)
        # 'ffmpeg' (padrão) ou 'opencv'. Ver _FFmpegCapture: o ffmpeg dobra a
        # taxa porque decodifica MJPEG multithread. O 'opencv' fica como saída
        # se o ffmpeg não estiver instalado na máquina da feira.
        self.declare_parameter('backend', 'ffmpeg')

        self.cfg, from_disk = cfgmod.load(logger=self.get_logger())
        dev = self.get_parameter('device').value
        if dev:
            self.cfg['device'] = dev

        self.get_logger().info(
            f"calibração {'carregada de ' + cfgmod.CONFIG_PATH if from_disk else 'DEFAULT (não calibrada!)'}")
        if not from_disk:
            self.get_logger().warn(
                'Sem vision.json: os cantos e as cores são chutes. '
                f'Calibre em http://localhost:{self.get_parameter("port").value}')

        self._rebuild()

        self._pub = self.create_publisher(GameData, '/game_data', 10)

        # O que a IA está pensando, desenhado por cima da imagem real.
        # Sem isto, rodar com `use_vision:=true` deixa o jogo sem nenhuma
        # janela para a decisão da IA: esse desenho existia só no simulador,
        # que é justamente a peça que a câmera substitui. E ele não é
        # depuração — é o que explica ao público, apontando para o campo, por
        # que o robô erra.
        self._ai = None
        self.create_subscription(AiDebug, '/ai/debug', self._on_ai_debug, 10)

        # Velocidades: uma trilha por grandeza que o GameData pede.
        a = float(self.get_parameter('vel_alpha').value)
        self._tr = {
            'ball_x': _Track(a), 'ball_y': _Track(a),
        }
        for rid in self.cfg['robot_ids']:
            self._tr[f'r{rid}_x'] = _Track(a)
            self._tr[f'r{rid}_y'] = _Track(a)
            self._tr[f'r{rid}_th'] = _Track(a, angular=True)

        self._frame = None            # último frame BGR cru
        self._overlay = None          # JPEG do overlay, para a GUI
        self._det = None
        self._lock = threading.Lock()
        self._stats = {'fps': 0.0, 'frames': 0, 'ball_miss': 0, 'robot_miss': 0}

        self._stop = threading.Event()
        self._cap_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._cap_thread.start()

        self._start_web()

    def _on_ai_debug(self, msg):
        self._ai = msg

    def _draw_ai(self, vis):
        """Alvo escolhido e bola percebida pela IA, em coordenadas de campo.

        O círculo tracejado é onde a IA **acha** que a bola está. Com atraso de
        reação ligado ele fica visivelmente atrás da bola real — em bola rápida
        chega a 40 cm. É a explicação visual de por que ela erra, sem precisar
        de palavra nenhuma.
        """
        ai = self._ai
        if ai is None:
            return

        try:
            (tx, ty), (bx, by) = self.detector.calib.to_pixels(
                [(ai.target_x, ai.target_y), (ai.ball_x, ai.ball_y)])
        except Exception:
            return

        t = (int(tx), int(ty))
        cv2.line(vis, (t[0] - 18, t[1]), (t[0] + 18, t[1]), (0, 220, 255), 2)
        cv2.line(vis, (t[0], t[1] - 18), (t[0], t[1] + 18), (0, 220, 255), 2)

        b = (int(bx), int(by))
        for a in range(0, 360, 30):        # tracejado: arcos alternados
            cv2.ellipse(vis, b, (26, 26), 0, a, a + 16, (255, 120, 255), 2)

        cv2.putText(vis, f'IA: {ai.state}  {ai.difficulty}  '
                         f'atraso {ai.reaction_delay*1000:.0f}ms  '
                         f'forca {ai.speed_frac*100:.0f}%',
                    (20, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255), 2)

    # ── configuração ────────────────────────────────────────────────────

    def _rebuild(self):
        calib, colors, ids = cfgmod.build(self.cfg)
        det = Detector(calib, colors, robot_ids=ids)
        if self.cfg.get('roi'):
            det.roi = tuple(self.cfg['roi'])
        self.detector = det

    # ── câmera ──────────────────────────────────────────────────────────

    def _apply_v4l2(self):
        """Trava exposição, ganho, balanço de branco e foco.

        Um a um, nesta ordem, e conferindo depois. Em lote falha: o
        `focus_absolute` fica inativo enquanto o autofoco está ligado e o driver
        responde `Permission denied` para o lote inteiro.

        Isto não é preciosismo: com os automáticos ligados, um objeto branco no
        campo (ou alguém de camisa clara passando na frente) reexpõe a câmera e
        derruba todos os limiares de cor de uma vez.
        """
        dev = self.cfg['device']
        v4l2 = self.cfg.get('v4l2') or dict(cfgmod.DEFAULT_V4L2)
        order = [k for k, _ in cfgmod.DEFAULT_V4L2 if k in v4l2]

        for name in order:
            val = v4l2[name]
            r = subprocess.run(['v4l2-ctl', '-d', dev, f'--set-ctrl={name}={val}'],
                               capture_output=True, text=True)
            if r.returncode != 0:
                self.get_logger().warn(f'v4l2 {name}={val}: {r.stderr.strip()}')

    def _verify_v4l2(self):
        """Relê os controles e reaplica o que voltou sozinho.

        O `gain` foi observado voltando para 8 depois que o stream abre. Ler de
        volta e reaplicar é a diferença entre um campo bem exposto e um campo
        escuro em que metade das cores some.
        """
        dev = self.cfg['device']
        v4l2 = self.cfg.get('v4l2') or dict(cfgmod.DEFAULT_V4L2)
        names = ','.join(v4l2)
        r = subprocess.run(['v4l2-ctl', '-d', dev, f'--get-ctrl={names}'],
                           capture_output=True, text=True)
        got = {}
        for line in r.stdout.splitlines():
            if ':' in line:
                k, v = line.split(':', 1)
                try:
                    got[k.strip()] = int(v.strip())
                except ValueError:
                    pass

        for name, want in v4l2.items():
            have = got.get(name)
            if have is None or have == want:
                continue
            self.get_logger().warn(f'{name} voltou para {have}, reaplicando {want}')
            subprocess.run(['v4l2-ctl', '-d', dev, f'--set-ctrl={name}={want}'],
                           capture_output=True, text=True)

            if name != 'exposure_time_absolute':
                continue

            # O exposure tem teto legítimo (1/fps), então "não bateu" pode ser
            # normal. O que NÃO é normal é ficar muito abaixo: aí o frame sai
            # escuro e a detecção morre silenciosamente. Vale avisar alto, com
            # a saída concreta, porque o sintoma (nada detectado) não aponta
            # para a causa sozinho.
            r2 = subprocess.run(
                ['v4l2-ctl', '-d', dev, '--get-ctrl=exposure_time_absolute'],
                capture_output=True, text=True)
            final = have
            if ':' in r2.stdout:
                try:
                    final = int(r2.stdout.split(':', 1)[1].strip())
                except ValueError:
                    pass
            if final < 0.75 * want:
                self.get_logger().error(
                    f'exposure travou em {final} (pedi {want}). O frame vai '
                    f'sair escuro e a detecção pode zerar. Compense subindo o '
                    f'gain em ~{int(100 * want / max(final, 1))}% na calibração.')
            else:
                self.get_logger().info(f'exposure em {final} — ok')

    def _open_camera(self):
        dev = self.cfg['device']
        w, h, fps = self.cfg['width'], self.cfg['height'], self.cfg['fps']

        backend = self.get_parameter('backend').value
        if backend == 'ffmpeg':
            cap = _FFmpegCapture(dev, w, h, fps)
        else:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError(f'não abriu {dev}')
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS, fps)
            # Fila de 1: numa visão, frame velho não vale nada.
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # A ordem aqui não é arbitrária e custou uma depuração: o teto do
        # `exposure_time_absolute` depende do intervalo de frame que o driver
        # negocia quando o streaming COMEÇA. Aplicar os controles antes do
        # primeiro `read()` faz o exposure ser cortado pelo teto do modo antigo
        # — vi ele grudar em 156 em vez de 312, o que escurece o frame pela
        # metade e derruba todos os limiares de cor de uma vez: nenhum robô,
        # nenhuma bola, e um /game_data vazio sem nenhum erro no log.
        #
        # Então: abre o stream primeiro, configura depois, confere no fim.
        for _ in range(5):
            cap.read()
        self._apply_v4l2()
        for _ in range(5):
            cap.read()
        self._verify_v4l2()

        self.get_logger().info(f'câmera {dev} em {w}x{h} (backend {backend})')
        return cap

    def _capture_loop(self):
        try:
            cap = self._open_camera()
        except Exception as exc:
            self.get_logger().error(f'câmera: {exc}')
            return

        t_fps, n_fps = time.time(), 0
        misses, blind_since = 0, None
        while not self._stop.is_set():
            ok, frame = cap.read()
            if not ok:
                # Falha de leitura quase nunca é um frame isolado: ou a câmera
                # foi desconectada, ou outro processo pegou o dispositivo. Sem
                # backoff isto vira loop quente logando ~90 vezes por segundo
                # — o que já encheu um log inteiro aqui e escondeu a causa real
                # no meio do ruído.
                misses += 1
                if blind_since is None:
                    blind_since = time.time()
                if misses in (1, 10, 100):
                    self.get_logger().warn(f'frame perdido ({misses}×)')

                blind = time.time() - blind_since
                if blind >= self.MAX_BLIND_SECONDS:
                    self.get_logger().error(
                        f'{blind:.0f}s sem nenhum frame — desistindo da câmera '
                        f'{self.cfg["device"]}. Confira se ela está conectada e '
                        f'se nenhum outro processo a segura '
                        f'(fuser -v {self.cfg["device"]}).')
                    break
                time.sleep(min(0.5, 0.01 * misses))
                continue
            misses, blind_since = 0, None

            t = time.time()
            det = self.detector.detect(frame)

            with self._lock:
                self._frame = frame
                self._det = det
            self._publish(det, t)

            n_fps += 1
            if t - t_fps >= 1.0:
                self._stats['fps'] = n_fps / (t - t_fps)
                t_fps, n_fps = t, 0

        cap.release()

    # ── publicação ──────────────────────────────────────────────────────

    def _publish(self, det, t):
        msg = GameData()
        msg.stamp = self.get_clock().now().to_msg()
        msg.ball_detected = det.ball_detected

        self._stats['frames'] += 1

        if det.ball_detected:
            bx, by = det.ball_m
            msg.ball.x = bx
            msg.ball.y = by
            msg.ball.vx = self._tr['ball_x'].update(bx, t)
            msg.ball.vy = self._tr['ball_y'].update(by, t)
        else:
            # Não seguro a última posição conhecida: `ball_detected=False` já
            # tem significado definido rio abaixo — a IA para e o game_master
            # ignora o frame. Inventar posição aqui seria mentir para os dois.
            self._stats['ball_miss'] += 1
            self._tr['ball_x'].reset()
            self._tr['ball_y'].reset()

        seen = set()
        for r in det.robots:
            st = RobotState()
            st.id = int(r.id)
            st.x, st.y = r.x, r.y
            st.orientation = r.theta
            st.vx = self._tr[f'r{r.id}_x'].update(r.x, t)
            st.vy = self._tr[f'r{r.id}_y'].update(r.y, t)
            st.vtheta = self._tr[f'r{r.id}_th'].update(r.theta, t)
            msg.robots.append(st)
            seen.add(r.id)

        for rid in self.cfg['robot_ids']:
            if rid not in seen:
                self._stats['robot_miss'] += 1
                for k in ('x', 'y', 'th'):
                    self._tr[f'r{rid}_{k}'].reset()

        self._pub.publish(msg)

    # ── GUI de calibração ───────────────────────────────────────────────

    def _draw_overlay(self, frame, det):
        vis = frame.copy()

        # As marcações do campo projetadas pela homografia. Se a calibração
        # estiver certa, elas caem em cima das linhas pintadas — e é assim que
        # alguém que nunca viu este código confere se acertou os cantos. Erro
        # em metros é difícil de julgar; linha do meio fora do lugar não é.
        marks = self.cfg.get('marks') or {}
        try:
            for poly in self.detector.calib.model_polylines(**marks):
                px = self.detector.calib.to_pixels(poly).astype(np.int32)
                cv2.polylines(vis, [px], False, (255, 190, 0), 2, cv2.LINE_AA)
        except Exception:
            pass                       # cantos degenerados: não derruba o stream

        pts = np.int32(self.detector.calib.corners_px)
        cv2.polylines(vis, [pts], True, (0, 255, 255), 2)
        for i, p in enumerate(pts):
            cv2.circle(vis, tuple(p), 10, (0, 255, 255), -1)
            cv2.putText(vis, str(i), tuple(p + np.array([14, -8])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

        if det and det.ball_px:
            c = tuple(np.int32(det.ball_px))
            cv2.circle(vis, c, 20, (0, 0, 255), 3)
            cv2.putText(vis, f'{det.ball_m[0]:+.2f},{det.ball_m[1]:+.2f}',
                        (c[0] + 24, c[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255), 2)

        for r in (det.robots if det else []):
            c = (int(r.px), int(r.py))
            cv2.circle(vis, c, 24, (0, 255, 0), 3)
            tip = (int(r.px + 55 * math.cos(r.theta)),
                   int(r.py - 55 * math.sin(r.theta)))
            cv2.arrowedLine(vis, c, tip, (0, 255, 0), 3, tipLength=0.3)
            cv2.putText(vis, f'{r.id}: {r.x:+.2f},{r.y:+.2f} '
                             f'{math.degrees(r.theta):+.0f}',
                        (c[0] + 28, c[1] - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0), 2)

        cv2.putText(vis, f"{self._stats['fps']:.1f} fps",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)
        self._draw_ai(vis)
        return vis

    def _start_web(self):
        thread = threading.Thread(target=self._run_web, daemon=True)
        thread.start()

    def _run_web(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        app = web.Application()
        app.router.add_get('/', self._serve_index)
        app.router.add_get('/stream.mjpg', self._serve_stream)
        app.router.add_get('/ws', self._ws)

        port = int(self.get_parameter('port').value)
        runner = web.AppRunner(app)
        try:
            loop.run_until_complete(runner.setup())
            loop.run_until_complete(web.TCPSite(runner, '0.0.0.0', port).start())
        except OSError as exc:
            # A GUI é para calibrar; a visão não depende dela para publicar.
            # Então perder a porta NÃO derruba o nó — mas tem que ser uma
            # decisão explícita e visível, não um traceback de thread daemon
            # que ninguém lê e que deixa o nó rodando meio vivo.
            self.get_logger().error(
                f'porta {port} indisponível ({exc}). A visão continua '
                f'publicando /game_data, mas a calibração fica sem GUI. '
                f'Provavelmente já há outro vision_node rodando.')
            return

        self.get_logger().info(f'calibração em http://localhost:{port}')
        loop.run_forever()

    async def _serve_index(self, request):
        share = get_package_share_directory('vision_game')
        return web.FileResponse(os.path.join(share, 'web', 'index.html'))

    async def _serve_stream(self, request):
        """MJPEG multipart — funciona em <img src> sem uma linha de JS."""
        resp = web.StreamResponse(headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame'})
        await resp.prepare(request)
        try:
            while not self._stop.is_set():
                with self._lock:
                    frame, det = self._frame, self._det
                if frame is None:
                    await asyncio.sleep(0.05)
                    continue
                vis = self._draw_overlay(frame, det)
                # A GUI é para calibrar, não para o público: metade da largura
                # é nítida o bastante e cabe folgado na rede.
                vis = cv2.resize(vis, (vis.shape[1] // 2, vis.shape[0] // 2))
                ok, jpg = cv2.imencode('.jpg', vis,
                                       [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    await resp.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                                     + jpg.tobytes() + b'\r\n')
                await asyncio.sleep(1 / 15)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp

    async def _ws(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        await ws.send_json({'type': 'config', 'config': self.cfg})

        async for m in ws:
            if m.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(m.data)
            except json.JSONDecodeError:
                continue

            kind = data.get('type')
            if kind == 'corners':
                self.cfg['corners_px'] = data['corners']
                self._rebuild()
            elif kind == 'color':
                self.cfg['colors'][data['name']].update(data['spec'])
                self._rebuild()
            elif kind == 'pick':
                await self._do_pick(ws, data)
            elif kind == 'relocate':
                await self._do_relocate(ws)
            elif kind == 'v4l2':
                name, val = data['name'], int(data['value'])
                self.cfg.setdefault('v4l2', {})[name] = val
                subprocess.run(['v4l2-ctl', '-d', self.cfg['device'],
                                f'--set-ctrl={name}={val}'], capture_output=True)
            elif kind == 'save':
                try:
                    cfgmod.save(self.cfg)
                    # Guarda a foto junto: é ela que permite reencontrar os
                    # cantos depois que a câmera mudar de lugar.
                    with self._lock:
                        frame = None if self._frame is None else self._frame.copy()
                    if frame is not None:
                        cv2.imwrite(cfgmod.REFERENCE_PATH, frame)
                    await ws.send_json({'type': 'saved'})
                    self.get_logger().info(
                        f'calibração salva em {cfgmod.CONFIG_PATH} '
                        f'(+ referência em {cfgmod.REFERENCE_PATH})')
                except Exception as exc:
                    await ws.send_json({'type': 'error', 'msg': str(exc)})
            elif kind == 'stats':
                await ws.send_json({'type': 'stats', 'stats': self._stats})
        return ws

    async def _do_pick(self, ws, data):
        """Clique numa cor no vídeo → faixa HSV derivada da amostra."""
        with self._lock:
            frame = None if self._frame is None else self._frame.copy()
        if frame is None:
            await ws.send_json({'type': 'error', 'msg': 'sem imagem ainda'})
            return

        name = data['name']
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        try:
            prev = self.detector.colors.get(name)
            spec = pick_color(hsv, int(data['x']), int(data['y']), name,
                              radius=int(data.get('radius', 6)), prev=prev)
        except Exception as exc:
            await ws.send_json({'type': 'error', 'msg': str(exc)})
            return

        self.cfg['colors'][name] = cfgmod._spec_to_dict(spec)
        self._rebuild()
        await ws.send_json({'type': 'config', 'config': self.cfg})
        self.get_logger().info(
            f'{name}: H {spec.h_lo}–{spec.h_hi}, S≥{spec.s_min}, V≥{spec.v_min}')

    async def _do_relocate(self, ws):
        """Reencontra os cantos alinhando o quadro de agora à foto guardada."""
        if not os.path.exists(cfgmod.REFERENCE_PATH):
            await ws.send_json({
                'type': 'error',
                'msg': 'sem foto de referência. Clique os cantos e salve uma '
                       'vez com capricho; daí em diante isto passa a funcionar.'})
            return

        ref = cv2.imread(cfgmod.REFERENCE_PATH)
        with self._lock:
            cur = None if self._frame is None else self._frame.copy()
        if ref is None or cur is None:
            await ws.send_json({'type': 'error', 'msg': 'sem imagem para comparar'})
            return

        corners, info = relocate_corners(ref, self.cfg['corners_px'], cur)
        if corners is None:
            await ws.send_json({'type': 'error', 'msg': f'não reencontrei: {info}'})
            self.get_logger().warn(f'relocate falhou: {info}')
            return

        self.cfg['corners_px'] = [list(p) for p in corners]
        self._rebuild()
        await ws.send_json({'type': 'config', 'config': self.cfg})
        await ws.send_json({
            'type': 'note',
            'msg': f'cantos reencontrados com {info} pontos coerentes — '
                   f'confira se as linhas azuis caíram sobre as pintadas'})
        self.get_logger().info(f'cantos reencontrados ({info} inliers)')

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
