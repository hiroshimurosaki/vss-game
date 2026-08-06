"""Detecção da bola e dos robôs num frame da câmera.

Não depende de ROS — dá para importar e testar direto num .jpg, do mesmo jeito
que `simulator/physics.py`. O nó só embrulha isto em mensagem.

O problema aqui é bem menor do que a literatura de VSS costuma tratar: são
**dois robôs, um por time**. Isso muda o algoritmo inteiro. Não preciso resolver
"qual dos três robôs amarelos é o 2", que é a parte cara e frágil. A cor do
retângulo já é a identidade, e o quadrado menor serve só para dar o ângulo.

Convenções de saída, ditadas por `shared_interfaces/GameData.msg`:

- metros, origem no **centro** do campo
- x ao longo do comprimento (1,50 m), y ao longo da largura (1,30 m)
- `orientation` em radianos, 0 = +x, crescendo no sentido anti-horário

O eixo y da imagem cresce para baixo e o do campo cresce para cima, então a
homografia já entrega y invertido — ver `_FIELD_CORNERS_M`.
"""

from dataclasses import dataclass, field as dc_field

import cv2
import numpy as np


# ── Cores ────────────────────────────────────────────────────────────────

@dataclass
class ColorSpec:
    """Uma faixa em HSV do OpenCV (H 0–179, S 0–255, V 0–255).

    `h_lo > h_hi` significa faixa que dá a volta no zero — é o caso do
    vermelho/laranja da bola, que fica em H≈175..179 e 0..8 ao mesmo tempo.
    Sem esse caso especial a bola pisca conforme a luz muda o matiz em um grau.
    """

    name: str
    h_lo: int
    h_hi: int
    s_min: int = 80
    v_min: int = 60
    s_max: int = 255
    v_max: int = 255
    min_area: int = 60        # px; abaixo disso é ruído de compressão MJPG
    max_area: int = 20000

    def mask(self, hsv: np.ndarray) -> np.ndarray:
        lo_s, hi_s = self.s_min, self.s_max
        lo_v, hi_v = self.v_min, self.v_max

        if self.h_lo <= self.h_hi:
            return cv2.inRange(hsv,
                               (self.h_lo, lo_s, lo_v),
                               (self.h_hi, hi_s, hi_v))

        # Faixa que cruza o zero: duas fatias, unidas.
        a = cv2.inRange(hsv, (self.h_lo, lo_s, lo_v), (179, hi_s, hi_v))
        b = cv2.inRange(hsv, (0, lo_s, lo_v), (self.h_hi, hi_s, hi_v))
        return cv2.bitwise_or(a, b)


# Ponto de partida medido no campo do laboratório, câmera C920 a 1080p com
# exposição e balanço de branco travados. NÃO são valores universais: a
# calibração salva em ~/.vss-game/vision.json sobrescreve todos eles.
DEFAULT_COLORS = {
    'ball':   ColorSpec('ball',   h_lo=170, h_hi=14, s_min=120, v_min=120,
                        min_area=180, max_area=3000),
    'team_a': ColorSpec('team_a', h_lo=15,  h_hi=38, s_min=55,  v_min=90,
                        min_area=150),
    'team_b': ColorSpec('team_b', h_lo=45,  h_hi=88, s_min=60,  v_min=40,
                        min_area=150),
    'front':  ColorSpec('front',  h_lo=94,  h_hi=112, s_min=85, v_min=105,
                        min_area=70),
}
# Estes números saíram de busca em grade contra posições reais anotadas à mão
# no campo, escolhendo o par (S,V) que ainda pega a etiqueta inteira com ZERO
# pixel falso no resto do campo. Não são chute, mas também não são universais:
# valem para esta câmera, esta lâmpada e este campo. Quem montar noutro lugar
# usa o clique da GUI (`pick_color`), que refaz tudo isto a partir de uma
# amostra.
#
# Duas armadilhas que estes valores contornam, e que só apareceram medindo:
#
# 1. `team_a` (amarelo) precisa de `s_min` BAIXO. Subir de 55 para 85 derruba
#    o blob de 1033 px para 186 px — o amarelo do papel é bem menos saturado
#    do que parece a olho.
#
# 2. `front` (azul-bebê) é o caso apertado, e por um motivo físico: sob esta
#    lâmpada **o feltro fica azulado** (H 105–114, S até 109 no percentil 90).
#    Isso invade o azul-bebê (H 102–105, S 99–112) em matiz E em saturação, e
#    o único separador que sobra é o brilho. Daí `v_min=105`, alto de
#    propósito. Se um dia trocarem a lâmpada por uma mais quente, este é o
#    primeiro limiar a rever — e o candidato natural é trocar o azul-bebê por
#    uma cor longe do azul.


def _hue_stats(hues: np.ndarray):
    """Média e dispersão de matiz, tratando a volta no zero.

    Média aritmética de matiz é errada e falha exatamente no caso que mais
    importa aqui: laranja/vermelho fica em H≈176 e H≈4 ao mesmo tempo, e a
    média ingênua dos dois dá 90 — verde. Somando vetores unitários no ângulo
    dobrado (H é 0–179, meia volta) isso não acontece.
    """
    ang = hues.astype(np.float64) * (2 * np.pi / 180.0)
    cx, cy = np.cos(ang).mean(), np.sin(ang).mean()
    mean = (np.arctan2(cy, cx) * 180.0 / (2 * np.pi)) % 180.0

    d = np.abs(hues - mean)
    d = np.minimum(d, 180.0 - d)          # distância pelo lado curto
    return mean, d


def pick_color(hsv: np.ndarray, x: int, y: int, name: str,
               radius: int = 6, prev: ColorSpec = None) -> ColorSpec:
    """Deriva uma faixa HSV a partir de um clique na imagem.

    É o que torna a calibração transferível. Os limiares que eu medi valem para
    um campo, uma lâmpada e uma câmera; clicar na bola funciona em qualquer
    campo, com qualquer cor de bola, sob qualquer luz — que é o requisito real.

    A faixa sai de percentis, não de mínimo e máximo: um clique quase sempre
    pega alguns pixels da borda do objeto, já misturados com o fundo, e deixar
    esses pixels definirem o limite abre a faixa até ela pegar o campo inteiro.
    """
    h, w = hsv.shape[:2]
    x0, x1 = max(0, x - radius), min(w, x + radius + 1)
    y0, y1 = max(0, y - radius), min(h, y + radius + 1)
    patch = hsv[y0:y1, x0:x1].reshape(-1, 3)
    if len(patch) < 9:
        raise ValueError('clique fora da imagem')

    H, S, V = patch[:, 0], patch[:, 1], patch[:, 2]
    mean_h, dev = _hue_stats(H)

    # Largura da faixa de matiz: o espalhamento real da amostra, com um piso
    # (a compressão MJPG sozinha já mexe um ou dois graus) e um teto (faixa
    # larga demais deixa de ser uma cor e passa a ser meio círculo cromático).
    half = float(np.clip(np.percentile(dev, 90) * 1.6, 5.0, 22.0))
    h_lo = int(round((mean_h - half) % 180))
    h_hi = int(round((mean_h + half) % 180))

    # S e V só precisam de um piso: o que separa etiqueta de feltro preto é a
    # saturação. Fico bem abaixo do observado para tolerar o mesmo objeto na
    # parte mais escura do campo — aqui a iluminação varia mais de 4×.
    s_min = int(max(40, np.percentile(S, 10) * 0.55))
    v_min = int(max(18, np.percentile(V, 10) * 0.45))

    base = prev or DEFAULT_COLORS.get(name)
    return ColorSpec(name=name, h_lo=h_lo, h_hi=h_hi,
                     s_min=s_min, v_min=v_min,
                     s_max=255, v_max=255,
                     min_area=base.min_area if base else 80,
                     max_area=base.max_area if base else 20000)


# ── Calibração geométrica ────────────────────────────────────────────────

@dataclass
class FieldCalib:
    """Homografia pixel → metro, a partir dos 4 cantos do campo.

    Os cantos são os do **retângulo de jogo**, não os do tabuleiro de madeira.
    O campo real tem os cantos chanfrados a 45° (para a bola não encalhar), e é
    por isso que não dá para achá-los procurando um quadrilátero na imagem: o
    contorno é um octógono. O jeito confiável é o humano clicar uma vez, na
    interseção virtual das linhas — a GUI de calibração desenha as retas
    estendidas para ajudar a mirar.

    Ordem dos cantos: começando pelo gol da ESQUERDA / lado de cima da imagem,
    seguindo no sentido horário.
    """

    corners_px: list                      # [(x,y)] × 4, em pixel
    length: float = 1.50                  # eixo x, metros
    width: float = 1.30                   # eixo y, metros
    _H: np.ndarray = dc_field(default=None, repr=False)

    def field_corners_m(self):
        hl, hw = self.length / 2.0, self.width / 2.0
        # Sentido horário na imagem = sentido horário aqui, com y do campo já
        # invertido em relação ao y da imagem.
        return np.float32([
            (-hl,  hw),   # gol esquerdo, lado de cima
            ( hl,  hw),   # gol direito,  lado de cima
            ( hl, -hw),   # gol direito,  lado de baixo
            (-hl, -hw),   # gol esquerdo, lado de baixo
        ])

    @property
    def H(self) -> np.ndarray:
        if self._H is None:
            src = np.float32(self.corners_px)
            self._H = cv2.getPerspectiveTransform(src, self.field_corners_m())
        return self._H

    def to_meters(self, pts_px) -> np.ndarray:
        """(N,2) em pixel → (N,2) em metro."""
        pts = np.asarray(pts_px, dtype=np.float32).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(pts, self.H).reshape(-1, 2)

    def to_pixels(self, pts_m) -> np.ndarray:
        pts = np.asarray(pts_m, dtype=np.float32).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(pts, np.linalg.inv(self.H)).reshape(-1, 2)

    def model_polylines(self, goal_width=0.40, area_depth=0.15,
                        area_width=0.70):
        """As marcações do campo, em metros, para desenhar por cima da imagem.

        É a conferência da calibração, e é o que permite alguém que nunca viu
        este código saber se acertou: se a homografia estiver certa, estas
        linhas caem **em cima** das linhas pintadas no campo de verdade. Se
        estiverem deslocadas ou tortas, os cantos foram clicados errado.

        Vale mais que qualquer número: erro de calibração é difícil de julgar
        em metros e óbvio quando a linha do meio não cai na linha do meio.
        """
        hl, hw = self.length / 2.0, self.width / 2.0
        ha, hg = area_width / 2.0, goal_width / 2.0

        out = [
            # borda do campo
            [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw), (-hl, -hw)],
            # linha de meio-campo
            [(0.0, -hw), (0.0, hw)],
        ]
        for s in (-1.0, 1.0):                     # grande área dos dois lados
            out.append([(s * hl, -ha),
                        (s * (hl - area_depth), -ha),
                        (s * (hl - area_depth), ha),
                        (s * hl, ha)])
            out.append([(s * hl, -hg), (s * hl, hg)])   # boca do gol
        return out


def relocate_corners(reference: np.ndarray, ref_corners, current: np.ndarray,
                     min_matches: int = 25):
    """Reencontra os cantos num quadro novo, alinhando-o ao de referência.

    Devolve `([(x,y)]×4, n_inliers)` ou `(None, motivo)`.

    Esta é a resposta certa para o caso real: a câmera, a lâmpada e o campo são
    sempre os mesmos, só a **posição** da câmera muda entre montagens. Então
    calibrar tudo de novo do zero a cada vez é trabalho jogado fora — basta
    descobrir como a câmera se moveu.

    Casa pontos entre a foto guardada na calibração e o quadro de agora,
    estima a homografia entre os dois e passa os cantos antigos por ela. O
    campo tem textura de sobra para isso: linhas, marcações e até os riscos do
    feltro.

    Por que não achar os cantos direto na imagem: as bordas laterais do campo
    são **interrompidas pela boca do gol**, então viram segmentos curtos,
    enquanto a linha da grande área é longa e contínua. Qualquer detector de
    reta prefere a área e erra o campo em ~10 cm. Testei, e é o que acontece.
    """
    g1 = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
    g2 = cv2.cvtColor(current, cv2.COLOR_BGR2GRAY)

    orb = cv2.ORB_create(nfeatures=4000)
    k1, d1 = orb.detectAndCompute(g1, None)
    k2, d2 = orb.detectAndCompute(g2, None)
    if d1 is None or d2 is None or len(k1) < min_matches or len(k2) < min_matches:
        return None, 'textura insuficiente na imagem'

    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    pairs = bf.knnMatch(d1, d2, k=2)
    # Razão de Lowe: descarta casamento cujo segundo melhor é quase tão bom
    # quanto o melhor, que é o padrão de quem casou com ruído.
    good = [m for m, n in (p for p in pairs if len(p) == 2)
            if m.distance < 0.75 * n.distance]
    if len(good) < min_matches:
        return None, f'só {len(good)} pontos casaram (mínimo {min_matches})'

    src = np.float32([k1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([k2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    M, mask = cv2.findHomography(src, dst, cv2.RANSAC, 4.0)
    if M is None:
        return None, 'não estimei a homografia entre os quadros'

    inliers = int(mask.sum()) if mask is not None else 0
    if inliers < min_matches:
        return None, f'só {inliers} pontos coerentes (mínimo {min_matches})'

    pts = np.float32(ref_corners).reshape(-1, 1, 2)
    out = cv2.perspectiveTransform(pts, M).reshape(-1, 2)

    h, w = current.shape[:2]
    if any(not (-0.15 * w <= x <= 1.15 * w and -0.15 * h <= y <= 1.15 * h)
           for x, y in out):
        return None, 'os cantos caíram fora do quadro'

    return [(int(round(x)), int(round(y))) for x, y in out], inliers


def auto_corners(frame: np.ndarray):
    """Chuta os 4 cantos do campo a partir das linhas brancas.

    ATENÇÃO: nesta geometria de campo isto erra — as bordas laterais são
    cortadas pela boca do gol e o ajuste trava na linha da grande área, uns
    10 cm para dentro. Fica como último recurso para quem não tem uma
    referência guardada ainda; a `relocate_corners` é o caminho bom.

    Devolve `[(x,y)] × 4` no sentido horário a partir do canto superior
    esquerdo, ou None se não achar as quatro bordas.

    Ajusta as quatro RETAS de borda e cruza duas a duas, em vez de procurar um
    quadrilátero no contorno. Isso importa porque o campo VSS tem os cantos
    chanfrados a 45° — o contorno é um octógono, e pedir quadrilátero a ele dá
    um quadrilátero torto. Cruzando retas, o canto sai na interseção virtual,
    que é exatamente o ponto que a homografia quer.

    É sugestão, não veredito: o resultado vai para a GUI para o humano
    conferir e ajustar. Errar aqui desloca o campo inteiro, então não é o tipo
    de coisa que se aceita sem olhar.
    """
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    S, V = hsv[:, :, 1], hsv[:, :, 2]

    # O tabuleiro é escuro e cobre boa parte do quadro; o entulho em volta não
    # é confiável. Pego o maior borrão escuro e trabalho só dentro dele.
    dark = ((V < np.percentile(V, 60)) & (S < 120)).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((31, 31), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(dark, 8)
    if n < 2:
        return None
    big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    board = (lab == big).astype(np.uint8)
    board = cv2.erode(board, np.ones((9, 9), np.uint8))

    # Linha branca = pouco saturada e mais clara que o feltro em volta.
    inside = V[board > 0]
    if inside.size < 1000:
        return None
    thr = np.percentile(inside, 92)
    lines = (((V > thr) & (S < 90)).astype(np.uint8) * 255) * board
    lines = cv2.morphologyEx(lines, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    segs = cv2.HoughLinesP(lines, 1, np.pi / 360, threshold=80,
                           minLineLength=int(0.18 * max(w, h)), maxLineGap=25)
    if segs is None:
        return None

    horiz, vert = [], []
    for x1, y1, x2, y2 in segs[:, 0]:
        ang = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        a = abs((ang + 90) % 180 - 90)
        if a < 25:
            horiz.append((x1, y1, x2, y2))
        elif a > 65:
            vert.append((x1, y1, x2, y2))

    if len(horiz) < 2 or len(vert) < 2:
        return None

    def extreme(group, axis, take_max):
        """Segmento mais externo do grupo, pelo centro no eixo dado."""
        key = (lambda s: (s[1] + s[3]) / 2) if axis == 'y' else \
              (lambda s: (s[0] + s[2]) / 2)
        return (max if take_max else min)(group, key=key)

    top, bottom = extreme(horiz, 'y', False), extreme(horiz, 'y', True)
    left, right = extreme(vert, 'x', False), extreme(vert, 'x', True)

    def cross(a, b):
        (x1, y1, x2, y2), (x3, y3, x4, y4) = a, b
        d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(d) < 1e-6:
            return None
        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d
        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d
        return (int(round(px)), int(round(py)))

    pts = [cross(top, left), cross(top, right),
           cross(bottom, right), cross(bottom, left)]
    if any(p is None for p in pts):
        return None
    # Canto fora do quadro é sinal de reta mal ajustada — melhor não sugerir
    # nada do que sugerir errado e o humano aceitar sem olhar.
    if any(not (-0.1 * w <= p[0] <= 1.1 * w and -0.1 * h <= p[1] <= 1.1 * h)
           for p in pts):
        return None
    return pts


# ── Resultado ────────────────────────────────────────────────────────────

@dataclass
class Blob:
    cx: float                 # centróide em pixel
    cy: float
    area: int


@dataclass
class RobotDet:
    id: int
    x: float                  # metros
    y: float
    theta: float              # radianos
    px: float                 # pixel, para desenhar o overlay
    py: float
    area: int


@dataclass
class Detection:
    ball_m: tuple = None      # (x, y) em metros, ou None se não achou
    ball_px: tuple = None
    robots: list = dc_field(default_factory=list)

    @property
    def ball_detected(self) -> bool:
        return self.ball_m is not None


# ── Detector ─────────────────────────────────────────────────────────────

class Detector:
    """Segmenta por cor, converte para metros e resolve a orientação.

    A orientação vem do vetor **retângulo → quadrado da frente**. É o dado mais
    barato e mais estável que existe nessa etiqueta: os dois centróides estão a
    ~1,5 cm um do outro, e o vetor entre eles é perpendicular à linha que divide
    a etiqueta. Tentar tirar ângulo da forma do blob do robô é bem pior — o cubo
    é quase simétrico e o ângulo pula 90° sozinho.
    """

    def __init__(self, calib: FieldCalib, colors=None, robot_ids=(0, 1),
                 blur=3, open_ksize=3):
        self.calib = calib
        self.colors = dict(colors or DEFAULT_COLORS)
        # robot_ids[0] é quem usa a cor 'team_a'; robot_ids[1] usa 'team_b'.
        self.robot_ids = tuple(robot_ids)
        self.blur = blur
        self._kernel = np.ones((open_ksize, open_ksize), np.uint8)
        self.roi = None       # (x0, y0, x1, y1); None = deriva dos cantos
        self._auto_roi = None

    def auto_roi(self, shape, margin_px=40):
        """Recorte do campo a partir dos cantos calibrados, com folga.

        Existe para não virar mais um parâmetro que alguém esquece de ajustar:
        calibrou os cantos, o recorte vem junto. A folga cobre o robô encostado
        na linha, cujo centróide fica dentro mas cuja etiqueta passa um pouco.

        Não é só desempenho. Sem recorte, o entulho da sala em volta do campo
        gera blobs coloridos que ganham dos verdadeiros em área — numa medição
        aqui, o robô foi parar em (+1,02, +0,29) m, fora do campo.
        """
        h, w = shape[:2]
        pts = np.asarray(self.calib.corners_px, dtype=float)
        x0 = max(0, int(pts[:, 0].min()) - margin_px)
        y0 = max(0, int(pts[:, 1].min()) - margin_px)
        x1 = min(w, int(pts[:, 0].max()) + margin_px)
        y1 = min(h, int(pts[:, 1].max()) + margin_px)
        return (x0, y0, x1, y1)

    # -- primitivas ------------------------------------------------------

    def _hsv(self, frame: np.ndarray) -> np.ndarray:
        if self.blur and self.blur >= 3:
            frame = cv2.GaussianBlur(frame, (self.blur, self.blur), 0)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    def _blobs(self, hsv: np.ndarray, spec: ColorSpec, limit=8, offset=(0, 0)):
        """Blobs de uma cor, do maior para o menor.

        `hsv` já vem recortado no campo — ver `detect()`. Recortar antes de
        segmentar, e não depois, é o que faz esta função caber no orçamento:
        morfologia e componentes conexos custam proporcional à área, e o campo
        ocupa cerca de metade do quadro. Fazendo na ordem errada o detector
        rodava a 20 Hz numa câmera de 30.
        """
        m = spec.mask(hsv)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self._kernel)

        n, _, stats, cent = cv2.connectedComponentsWithStats(m, 8)
        ox, oy = offset
        out = []
        for i in range(1, n):
            a = int(stats[i, cv2.CC_STAT_AREA])
            if spec.min_area <= a <= spec.max_area:
                out.append(Blob(float(cent[i][0]) + ox,
                                float(cent[i][1]) + oy, a))
        out.sort(key=lambda b: -b.area)
        return out[:limit]

    # -- alvos -----------------------------------------------------------

    def _find_ball(self, hsv, robot_pts, offset=(0, 0)):
        """A bola é o maior blob laranja que **não** está colado num robô.

        Esse filtro existe por um motivo medido, não teórico: um quadrado de ID
        vermelho na etiqueta tem exatamente o mesmo matiz e a mesma saturação da
        bola. Só o brilho separa os dois, e o brilho varia ~3,6× ao longo do
        campo, então limiar de V sozinho não aguenta. Descartar candidatos que
        caem em cima de um robô resolve pela geometria, que é confiável.
        """
        cands = self._blobs(hsv, self.colors['ball'], limit=6, offset=offset)
        if not cands:
            return None

        hl = self.calib.length / 2.0 + self.out_of_field_margin
        hw = self.calib.width / 2.0 + self.out_of_field_margin

        for b in cands:
            if any(np.hypot(b.cx - rx, b.cy - ry) <= self.robot_exclusion_px
                   for rx, ry in robot_pts):
                continue

            # Candidato fora do campo não é a bola, ponto final. Sem esta
            # checagem, com a bola fora de campo o detector agarra qualquer
            # coisa avermelhada no cenário — vi ele travar numa viga do teto e
            # reportar "bola" com toda a confiança. A IA persegue o fantasma e
            # o game_master pode apitar em cima dele.
            #
            # É verificação em METROS, depois da homografia, e não recorte em
            # pixel: é a única que continua valendo quando a câmera muda de
            # lugar, que aqui acontece toda montagem.
            (bx, by), = self.calib.to_meters([(b.cx, b.cy)])
            if abs(bx) > hl or abs(by) > hw:
                continue
            return b
        return None

    def _find_robot(self, hsv, team_key, robot_id, front_blobs, offset=(0, 0)):
        body = self._blobs(hsv, self.colors[team_key], limit=1, offset=offset)
        if not body:
            return None
        b = body[0]

        # O quadrado da frente é o 'front' mais próximo deste retângulo. Com
        # dois robôs em campo há dois candidatos; o mais próximo é o certo,
        # porque eles estão a ~1,5 cm do próprio corpo e a dezenas de cm um do
        # outro.
        best, best_d = None, 1e9
        for f in front_blobs:
            d = np.hypot(f.cx - b.cx, f.cy - b.cy)
            # Um candidato praticamente em cima do centróide do retângulo não é
            # o quadrado da frente: é o próprio retângulo vazando para dentro do
            # mask da frente, porque as duas cores estão perto demais em matiz.
            # Aconteceu de verdade com azul-escuro (H≈107) contra azul-bebê
            # (H≈98) — 86% dos pixels em comum, vetor de comprimento 0,6 px e
            # ângulo aleatório. Sem esta guarda o robô anda de ré.
            if d < self.front_min_px:
                continue
            if d < best_d:
                best, best_d = f, d

        if best is None or best_d > self.front_max_px:
            return None       # sem orientação confiável, não invento ângulo

        (x, y), = self.calib.to_meters([(b.cx, b.cy)])
        (fx, fy), = self.calib.to_meters([(best.cx, best.cy)])
        theta = float(np.arctan2(fy - y, fx - x))

        return RobotDet(id=robot_id, x=float(x), y=float(y), theta=theta,
                        px=b.cx, py=b.cy, area=b.area)

    # -- API -------------------------------------------------------------

    #: Raio, em pixel, em que um candidato a bola é considerado "dentro" de um
    #: robô. ~2,5 cm a 1080p neste campo; a GUI de calibração ajusta.
    robot_exclusion_px = 34

    #: Distância máxima retângulo→quadrado para aceitar a orientação.
    front_max_px = 40

    #: Distância MÍNIMA. Abaixo disto o "quadrado da frente" é o próprio corpo
    #: vazando entre máscaras, e o ângulo resultante é ruído puro.
    front_min_px = 8

    #: Folga além da linha, em metros, ainda aceita como bola. A bola sai de
    #: campo o tempo todo e continua sendo a bola; o que se quer descartar é o
    #: objeto vermelho do cenário, que está muito mais longe que isto.
    out_of_field_margin = 0.12

    def detect(self, frame: np.ndarray) -> Detection:
        # Cache simples: o nó reconstrói o Detector inteiro quando a calibração
        # muda, então não existe caso em que este valor fique velho.
        if self.roi is None and self._auto_roi is None:
            self._auto_roi = self.auto_roi(frame.shape)
        roi = self.roi if self.roi is not None else self._auto_roi
        x0, y0, x1, y1 = roi
        frame = frame[y0:y1, x0:x1]
        off = (x0, y0)

        hsv = self._hsv(frame)
        fronts = self._blobs(hsv, self.colors['front'], limit=6, offset=off)

        robots = []
        for key, rid in (('team_a', self.robot_ids[0]),
                         ('team_b', self.robot_ids[1])):
            r = self._find_robot(hsv, key, rid, fronts, off)
            if r is not None:
                robots.append(r)

        ball = self._find_ball(hsv, [(r.px, r.py) for r in robots], off)

        det = Detection(robots=robots)
        if ball is not None:
            det.ball_px = (ball.cx, ball.cy)
            (bx, by), = self.calib.to_meters([(ball.cx, ball.cy)])
            det.ball_m = (float(bx), float(by))
        return det
