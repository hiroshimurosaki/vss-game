#!/usr/bin/env bash
# Grava os firmwares nas duas placas, no modo certo, sem você ter que descobrir
# qual /dev/ttyUSB é qual.
#
#   ./tools/gravar.sh feira    ponte=tx_bridge  robô=robot_rx   SEM debug
#   ./tools/gravar.sh debug    ponte=tx_bridge  robô=robot_rx   COM debug
#   ./tools/gravar.sh probe    ponte=tx_probe   robô=robot_rx   COM debug
#
#   ./tools/gravar.sh debug --id 0     grava o robô como robô 0 (o da IA)
#
# QUAL MODO USAR
#   feira  é o que joga. Debug ligado gasta tempo de serial dentro do loop e
#          pode perder pacote — não é o que você quer com público na frente.
#   debug  é o que o `tools/debug_panel.py` precisa: sem os dois lados falando,
#          o painel não tem o que ler e fica cego.
#   probe  troca a ponte pelo `tx_probe`, que reporta a taxa de entrega
#          confirmada por hardware. Serve para medir o link, não para jogar.
#
# POR QUE ELE DESCOBRE AS PLACAS PELO BANNER
#   As duas são CH340 sem número de série, então `/dev/serial/by-id/` cria um
#   link só, e qual das duas ele aponta é sorteio a cada replug. Gravar o
#   `tx_bridge` dentro do robô é fácil e custa caro. Ler o banner de boot é o
#   único jeito confiável — e só sai se a abertura da serial resetar a placa,
#   daí o `hupcl` abaixo.

set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH="$HOME/.local/bin:$PATH"
FQBN=arduino:avr:nano:cpu=atmega328

MODO="${1:-}"
shift || true

ROBOT_ID=1
PONTE_FORCADA=""
ROBO_FORCADO=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --id)    ROBOT_ID="$2"; shift 2 ;;
        --ponte) PONTE_FORCADA="$2"; shift 2 ;;
        --robo)  ROBO_FORCADO="$2"; shift 2 ;;
        *) echo "opção desconhecida: $1" >&2; exit 2 ;;
    esac
done

case "$MODO" in
    feira|debug|probe) ;;
    *) sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 2 ;;
esac

command -v arduino-cli >/dev/null || { echo "arduino-cli não está no PATH"; exit 1; }

# ── quem é quem ─────────────────────────────────────────────────────────
banner_de() {
    local dev="$1" tmp
    tmp="$(mktemp)"
    stty -F "$dev" 115200 raw -echo hupcl 2>/dev/null || { rm -f "$tmp"; return 1; }
    timeout 7 cat "$dev" > "$tmp" 2>/dev/null
    tr -d '\r' < "$tmp"
    rm -f "$tmp"
}

PONTE="$PONTE_FORCADA"
ROBO="$ROBO_FORCADO"

if [[ -z "$PONTE" || -z "$ROBO" ]]; then
    echo "descobrindo as placas pelo banner (reseta as duas, ~7s)..."

    for dev in /dev/ttyUSB*; do
        [[ -e "$dev" ]] || continue
        [[ "$dev" == "$PONTE_FORCADA" || "$dev" == "$ROBO_FORCADO" ]] && continue

        b="$(banner_de "$dev")"

        case "$b" in
            *tx_bridge*|*PROBE*)  [[ -z "$PONTE" ]] && PONTE="$dev"
                                  echo "  $dev -> ponte" ;;
            *robot_rx*|*sizeof*)  [[ -z "$ROBO" ]] && ROBO="$dev"
                                  echo "  $dev -> robô" ;;
            *)                    echo "  $dev -> não identifiquei (banner: ${b:-vazio})" ;;
        esac
    done
fi

if [[ -z "$PONTE" || -z "$ROBO" ]]; then
    echo
    echo "não achei as duas placas."
    echo "  ponte: ${PONTE:-NÃO ACHEI}   robô: ${ROBO:-NÃO ACHEI}"
    echo
    echo "Placa muda não quer dizer placa morta: um firmware que não imprime no"
    echo "boot fica calado. Force na mão se souber quem é quem:"
    echo "  $0 $MODO --ponte /dev/ttyUSBx --robo /dev/ttyUSBy"
    exit 1
fi

# ── o que gravar em cada uma ────────────────────────────────────────────
case "$MODO" in
    feira) SKETCH_PONTE=tx_bridge; FLAGS_PONTE="";              FLAGS_ROBO="" ;;
    debug) SKETCH_PONTE=tx_bridge; FLAGS_PONTE="-DDEBUG_TX=1";  FLAGS_ROBO="-DDEBUG_RADIO=1" ;;
    probe) SKETCH_PONTE=tx_probe;  FLAGS_PONTE="";              FLAGS_ROBO="-DDEBUG_RADIO=1" ;;
esac

grava() {
    local sketch="$1" porta="$2" extra="$3" rotulo="$4"
    local props=()

    # Aspas vazias em --build-property fazem o arduino-cli reclamar, então a
    # flag só entra quando existe.
    [[ -n "$extra" ]] && props=(--build-property "compiler.cpp.extra_flags=$extra")

    echo
    echo "### $rotulo: $sketch ${extra:-(sem debug)} -> $porta"

    if ! arduino-cli compile --fqbn "$FQBN" "${props[@]}" \
             -u -p "$porta" "$WS/firmware/$sketch" 2>&1 | tail -3; then
        echo "  FALHOU"
        return 1
    fi
}

# O robô precisa do MY_ROBOT_ID dele, que é #define no fonte. Em vez de editar o
# arquivo (e deixar o repo sujo), sobrescreve pelo compilador.
grava "$SKETCH_PONTE" "$PONTE" "$FLAGS_PONTE" "ponte" || exit 1
sleep 2
grava robot_rx "$ROBO" "$FLAGS_ROBO -DMY_ROBOT_ID=$ROBOT_ID" "robô (id $ROBOT_ID)" || exit 1

sleep 3
echo
echo "### conferindo os banners"
for d in "$PONTE" "$ROBO"; do
    echo "  $d:"
    banner_de "$d" | sed 's/^/    /'
done

echo
case "$MODO" in
    feira) echo "Pronto para jogar. Para o debug_panel enxergar, use: $0 debug" ;;
    debug) echo "Pronto para o ./tools/painel.py. ANTES DA FEIRA: $0 feira" ;;
    probe) echo "Ponte medindo entrega. Para voltar ao jogo: $0 feira" ;;
esac
