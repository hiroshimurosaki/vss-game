"""Calibração persistida em ~/.vss-game/vision.json.

Mesma pasta do `highscores.json`, e mesma disciplina: grava em arquivo
temporário e move por cima, para que faltar energia no meio da escrita não
deixe um JSON pela metade. Arquivo ilegível vira aviso e default — a visão
nunca deixa de subir por causa de calibração corrompida, porque na feira isso
significa o estande parado.
"""

import json
import os
import tempfile

from .detector import ColorSpec, DEFAULT_COLORS, FieldCalib

CONFIG_DIR = os.path.expanduser('~/.vss-game')
CONFIG_PATH = os.path.join(CONFIG_DIR, 'vision.json')

#: Foto do campo guardada junto com a calibração. Serve para reencontrar os
#: cantos quando a câmera muda de lugar: alinha-se o quadro de agora contra
#: esta foto e transportam-se os cantos. Ver `detector.relocate_corners`.
REFERENCE_PATH = os.path.join(CONFIG_DIR, 'vision_ref.png')

#: Controles v4l2 travados na abertura da câmera. A ordem importa e os valores
#: são o resultado de medição neste campo, não chute:
#:
#: - `exposure_time_absolute` satura em ~312 a 30 fps (o teto é 1/30 s). Pedir
#:   650 e ler 312 de volta é o comportamento normal, não um erro.
#: - por isso quem controla o brilho aqui é o **gain**, não o exposure.
#: - `focus_absolute` só aceita escrita DEPOIS que o autofoco é desligado;
#:   mandar tudo num `--set-ctrl` só dá `Permission denied`.
#: - o gain foi observado voltando sozinho para 8 entre aberturas do stream,
#:   então o nó reconfere com `--get-ctrl` depois de abrir e reaplica.
#:
#: Os valores abaixo são de um campo COM luminária dedicada. Com a luz só do
#: ambiente eram `exposure=300, gain=255` — no talo dos dois. Se a detecção
#: sumir depois de mexerem na iluminação, é aqui que se mexe primeiro, e a
#: ordem é: primeiro sobe o exposure (é luz de graça, só limitada pelos 30 fps),
#: e só depois o gain, que é o que traz ruído junto.
DEFAULT_V4L2 = [
    ('auto_exposure', 1),                 # 1 = manual
    ('exposure_time_absolute', 150),
    ('white_balance_automatic', 0),
    ('white_balance_temperature', 4000),
    ('focus_automatic_continuous', 0),
    ('focus_absolute', 0),
    ('backlight_compensation', 0),
    ('gain', 0),
]


def default_config() -> dict:
    return {
        'device': '/dev/video2',
        'width': 1920,
        'height': 1080,
        'fps': 30,
        'corners_px': [[424, 55], [1430, 85], [1459, 966], [373, 960]],
        'field': {'length': 1.50, 'width': 1.30},
        # Só para desenhar a conferência por cima da imagem — não entram em
        # nenhuma conta de posição.
        'marks': {'goal_width': 0.40, 'area_depth': 0.16, 'area_width': 0.69},
        'roi': None,
        'robot_ids': [0, 1],
        'v4l2': dict(DEFAULT_V4L2),
        'colors': {k: _spec_to_dict(v) for k, v in DEFAULT_COLORS.items()},
    }


def _spec_to_dict(s: ColorSpec) -> dict:
    return {'h_lo': s.h_lo, 'h_hi': s.h_hi,
            's_min': s.s_min, 's_max': s.s_max,
            'v_min': s.v_min, 'v_max': s.v_max,
            'min_area': s.min_area, 'max_area': s.max_area}


def _dict_to_spec(name: str, d: dict) -> ColorSpec:
    return ColorSpec(name=name, **d)


def load(path=CONFIG_PATH, logger=None):
    """Devolve (config_dict, veio_do_disco)."""
    if not os.path.exists(path):
        return default_config(), False
    try:
        with open(path) as f:
            data = json.load(f)
    except Exception as exc:
        if logger:
            logger.warn(f'vision.json ilegível ({exc}); usando o default')
        return default_config(), False

    cfg = default_config()
    cfg.update(data)                       # merge raso: chave nova não some
    return cfg, True


def save(cfg: dict, path=CONFIG_PATH):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(cfg, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def build(cfg: dict):
    """config dict → (FieldCalib, colors, robot_ids)."""
    calib = FieldCalib(corners_px=[tuple(p) for p in cfg['corners_px']],
                       length=cfg['field']['length'],
                       width=cfg['field']['width'])
    colors = {k: _dict_to_spec(k, v) for k, v in cfg['colors'].items()}
    return calib, colors, tuple(cfg['robot_ids'])
