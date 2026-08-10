#!/usr/bin/env python3
"""Recorta as fontes da marca para `web/fonts/*.woff2`.

    ./tools/make_fonts.py ~/Documentos/Carrossel/Design/Fontes-*/Fontes

As fontes ficam versionadas no repositório de propósito: a feira não tem rede
garantida, e fonte que não carrega derruba a identidade do telão na hora
errada. O que este script faz é tornar isso reproduzível — antes os `.woff2`
apareceram no repo sem nada que dissesse de onde vieram nem com que recorte.

QUEM É QUEM

    Advent Pro   é a letra do wordmark do manual da marca. Veio como fonte
                 VARIÁVEL, com eixos `wdth` (75–125) e `wght` (100–900), e é
                 assim que ela entra aqui: um arquivo só serve o wordmark
                 (largura normal), o placar do telão (expandida e pesada) e as
                 etiquetas (estreita). Instanciar cada largura num arquivo
                 separado custaria três vezes mais bytes pelo mesmo resultado.

    Poppins      é o corpo, os títulos e o placar. A revisão de design do site
                 trocou a Bowlby One por ela "para trazer mais seriedade", e o
                 telão segue a mesma decisão. Quatro pesos: 400 texto, 600
                 título, 700 ênfase, 900 placar.

    Fira Mono    fica para telemetria e números de ferramenta. É a única
                 sobrevivente da tipografia anterior, e sobrevive porque
                 monoespaçada é requisito funcional, não estético: coluna de
                 número a 30 Hz não pode tremer.

O RECORTE

    Latim básico + suplemento cobre todo o português, mais a pontuação da
    interface. Nada além disso, e por um motivo medido:

        cobertura            Advent Pro   Poppins   Fira Mono
        setas  U+2190-21FF        0/112     0/112      15/112
        caixa  U+2500-257F        0/128     0/128     116/128
        geom   U+25A0-25FF         0/96      2/96      55/96
        ✓ ✗    U+2713-2717          0/5       0/5         0/5
        ⚠      U+26A0               0/1       0/1         0/1

    Ou seja: NENHUMA fonte da marca tem seta, check, aviso ou bolinha. Pedir
    esses pontos no subset não os cria — só produz um arquivo que promete o que
    não entrega, e o navegador cai numa fonte do sistema para cada símbolo. Num
    computador de feira que ninguém auditou, isso é retângulo vazio no telão.

    Por isso a regra do projeto: **símbolo é SVG embutido, não caractere.**
    Seta, estado, check e aviso vivem no `vss.css` como ícone; texto é texto.
    O que sobreviveu porque a fonte realmente tem: × · • – — ° º ª (conferido
    na Advent Pro, que é quem desenha o placar).

Precisa de `fonttools` e `brotli`:  pip install 'fonttools[woff]' brotli
"""

import argparse
import pathlib
import shutil
import subprocess
import sys

# Latim básico + suplemento (todo o português) e a pontuação da interface.
# Símbolo NÃO entra aqui — ver "O RECORTE" no topo. É SVG no vss.css.
UNICODES = ','.join([
    'U+0000-00FF',        # ASCII + latim-1: acentuação do português inteira,
                          # e junto vem o × do placar e o ° da telemetria
    'U+0131',             # ı  — vem junto em fontes latinas, custa nada
    'U+0152-0153',        # Œ œ
    'U+02BB-02BC,U+02C6,U+02DA,U+02DC',
    'U+0300-0304,U+0308,U+0327,U+0329',   # combinantes
    'U+2000-206F',        # espaços, travessões, aspas tipográficas, …
    'U+2122',             # ™
    'U+FEFF,U+FFFD',
])

# nome de saída, arquivo de origem, argumentos extras do subset
JOBS = [
    # A variável entra inteira nos dois eixos: o fontTools.subset preserva
    # `fvar`/`gvar` por padrão. Quem achataria na instância padrão é o
    # `varLib.instancer`, que NÃO é usado aqui de propósito — é o que permite
    # um arquivo só servir wordmark, placar expandido e etiqueta estreita.
    ('display.woff2',       'AdventPro-VariableFont_wdth,wght.ttf', []),
    ('sans-regular.woff2',  'Poppins-Regular.ttf',  []),
    ('sans-semibold.woff2', 'Poppins-SemiBold.ttf', []),
    ('sans-bold.woff2',     'Poppins-Bold.ttf',     []),
    # O placar. Peso 900 num tamanho grande é o que dá contador aberto e haste
    # grossa a dez metros — é o numeral mais legível que a marca tem.
    ('sans-black.woff2',    'Poppins-Black.ttf',    []),
    ('mono-medium.woff2',   'FiraMono-Medium.otf',  []),
]

# A Fira Mono não está no pacote do manual da marca: vem do sistema, do pacote
# Debian `fonts-firacode`. Procurada aqui se não estiver no diretório de origem.
FALLBACK_DIRS = [
    pathlib.Path('/usr/share/fonts/opentype/fira'),
    pathlib.Path('/usr/share/fonts/truetype/fira'),
]


def find_source(name: str, src: pathlib.Path) -> pathlib.Path | None:
    for base in [src, *FALLBACK_DIRS]:
        hit = base / name
        if hit.exists():
            return hit
        # A Fira do apt varia de nome entre versões; cai no glob.
        stem = name.split('.')[0]
        for cand in base.glob(f'{stem}.*'):
            return cand
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source', type=pathlib.Path,
                    help='diretório com os .ttf/.otf de origem')
    ap.add_argument('--out', type=pathlib.Path,
                    default=pathlib.Path(__file__).resolve().parent.parent / 'web' / 'fonts',
                    help='destino (padrão: web/fonts)')
    args = ap.parse_args()

    # `python3 -m fontTools.subset` em vez do executável `pyftsubset`: o pacote
    # instalado via apt não põe o script no PATH, e o módulo é o mesmo código.
    try:
        import fontTools.subset  # noqa: F401
        import brotli            # noqa: F401
    except ImportError as e:
        print(f'falta {e.name}: pip install "fonttools[woff]" brotli', file=sys.stderr)
        return 1
    subset = [sys.executable, '-m', 'fontTools.subset']

    args.out.mkdir(parents=True, exist_ok=True)
    faltando = []

    for out_name, src_name, extra in JOBS:
        src = find_source(src_name, args.source)
        if src is None:
            faltando.append(src_name)
            continue

        dest = args.out / out_name
        cmd = [
            *subset, str(src),
            f'--output-file={dest}',
            '--flavor=woff2',
            f'--unicodes={UNICODES}',
            '--layout-features=kern,liga,tnum,frac',
            '--desubroutinize',
            '--no-hinting',
            '--drop-tables+=DSIG',
            *extra,
        ]
        subprocess.run(cmd, check=True)
        kb = dest.stat().st_size / 1024
        print(f'  {out_name:22s} {kb:6.1f} KB   ← {src.name}')

    if faltando:
        print('\nnão encontrados em', args.source, 'nem no sistema:', file=sys.stderr)
        for n in faltando:
            print('  -', n, file=sys.stderr)
        return 1

    print(f'\ntotal: {sum(f.stat().st_size for f in args.out.glob("*.woff2")) / 1024:.1f} KB')
    return 0


if __name__ == '__main__':
    sys.exit(main())
