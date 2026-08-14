#!/usr/bin/env bash
# Grava os firmwares nas duas placas, no modo certo, sem você ter que descobrir
# qual porta é qual.
#
#   ./tools/gravar.sh feira    ponte=tx_bridge  robô=robot_rx   SEM debug
#   ./tools/gravar.sh debug    ponte=tx_bridge  robô=robot_rx   COM debug
#   ./tools/gravar.sh probe    ponte=tx_probe   robô=robot_rx   COM debug
#
#   ./tools/gravar.sh debug --id 0     grava o robô como robô 0 (o da IA)
#
#   --so-ponte            grava só a ponte, sem exigir um robô plugado. É como
#                         se grava uma ponte sobressalente — inclusive em Arduino
#                         Uno, que serve de ponte sem mudar uma linha do
#                         firmware (mesmos pinos, mesmo 328p). Placa virgem não
#                         imprime banner, então a primeira gravação precisa da
#                         porta na mão: --so-ponte --ponte /dev/ttyACM0
#
#   --roda-a-sem-frente   paliativo para robô cuja roda A só gira para trás:
#                         troca a frente do robô e satura o que ela não entrega.
#                         Recupera andar para frente e girar para a esquerda,
#                         custa a ré. Só para robô avariado — ver o comentário
#                         em firmware/robot_rx/robot_rx.ino. Se for jogar com
#                         câmera, gire as etiquetas 180° no chassi junto.
#
#   --inverte-roda-a      robô com os dois fios do motor A trocados no TB6612:
#                         frente e ré viram giro para lados opostos, e girar
#                         anda reto. Inverter o sinal é exato e não custa
#                         movimento nenhum — mas o conserto de verdade é trocar
#                         os fios no chassi e gravar sem a flag.
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

# O FQBN sai do nome do device, não de um valor fixo: a ponte pode ser um Nano
# ou um Uno (ver abaixo), e as duas convivem.
#
# Medido em boards.txt do core arduino:avr 1.8.8: `uno` e `nano:cpu=atmega328`
# têm o MESMO mcu (atmega328p), o MESMO bootloader (optiboot), o MESMO protocolo
# (arduino) e a MESMA velocidade (115200). Só o maximum_size difere — 32256 no
# Uno contra 30720 no Nano. Ou seja: errar entre esses dois não impede a
# gravação, o binário é o mesmo. Por isso o fallback é seguro, e por isso o
# `atmega328old` (57600) continua sendo a única troca que realmente quebra.
fqbn_de() {
    case "$1" in
        # ttyACM = CDC do ATmega16U2, que só existe no Uno oficial. CH340 (Nano
        # e Uno clone) nunca chega aqui.
        /dev/ttyACM*) echo "arduino:avr:uno" ;;
        *)            echo "arduino:avr:nano:cpu=atmega328" ;;
    esac
}

MODO="${1:-}"
shift || true

ROBOT_ID=1
PONTE_FORCADA=""
ROBO_FORCADO=""
COMPENSA=""
SO_PONTE=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --id)    ROBOT_ID="$2"; shift 2 ;;
        --ponte) PONTE_FORCADA="$2"; shift 2 ;;
        --robo)  ROBO_FORCADO="$2"; shift 2 ;;
        # Grava só a ponte, sem exigir um robô plugado. Existe para a ponte
        # sobressalente: sem isto, gravar uma segunda ponte obriga a plugar um
        # robô que não tem nada a ver com a tarefa.
        --so-ponte) SO_PONTE=1; shift ;;
        # Acumulam: as duas compensações são independentes e podem coexistir
        # (a inversão normaliza o sinal antes de a outra decidir o que saturar).
        --roda-a-sem-frente) COMPENSA="$COMPENSA -DRODA_A_SEM_FRENTE=1"; shift ;;
        --inverte-roda-a)    COMPENSA="$COMPENSA -DINVERTE_RODA_A=1"; shift ;;
        *) echo "opção desconhecida: $1" >&2; exit 2 ;;
    esac
done

case "$MODO" in
    feira|debug|probe) ;;
    *) sed -n '2,37p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 2 ;;
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

if [[ -z "$PONTE" || ( -z "$ROBO" && "$SO_PONTE" == 0 ) ]]; then
    echo "descobrindo as placas pelo banner (reseta cada uma, ~7s)..."

    # ttyACM entra na varredura por causa da ponte em Uno oficial — ver o
    # comentário do fqbn_de(). O reset por DTR (o `hupcl` do banner_de) funciona
    # igual nos dois: o 16U2 reseta o 328p e não re-enumera, então o nome da
    # porta sobrevive ao reset e à gravação.
    for dev in /dev/ttyUSB* /dev/ttyACM*; do
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

if [[ -z "$PONTE" || ( -z "$ROBO" && "$SO_PONTE" == 0 ) ]]; then
    echo
    echo "não achei as placas que preciso."
    if [[ "$SO_PONTE" == 1 ]]; then
        echo "  ponte: ${PONTE:-NÃO ACHEI}   (--so-ponte: robô não é exigido)"
    else
        echo "  ponte: ${PONTE:-NÃO ACHEI}   robô: ${ROBO:-NÃO ACHEI}"
    fi
    echo
    echo "Placa muda não quer dizer placa morta: um firmware que não imprime no"
    echo "boot fica calado — e uma placa nova, de fábrica, é justamente assim."
    echo "Force na mão se souber quem é quem:"
    echo "  $0 $MODO --ponte /dev/ttyUSBx --robo /dev/ttyUSBy"
    echo "  $0 $MODO --so-ponte --ponte /dev/ttyACM0     # Uno oficial virgem"
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
    local fqbn
    fqbn="$(fqbn_de "$porta")"

    # Aspas vazias em --build-property fazem o arduino-cli reclamar, então a
    # flag só entra quando existe.
    [[ -n "$extra" ]] && props=(--build-property "compiler.cpp.extra_flags=$extra")

    echo
    echo "### $rotulo: $sketch ${extra:-(sem debug)} -> $porta [$fqbn]"

    if ! arduino-cli compile --fqbn "$fqbn" "${props[@]}" \
             -u -p "$porta" "$WS/firmware/$sketch" 2>&1 | tail -3; then
        echo "  FALHOU"
        return 1
    fi
}

# O robô precisa do MY_ROBOT_ID dele, que é #define no fonte. Em vez de editar o
# arquivo (e deixar o repo sujo), sobrescreve pelo compilador.
grava "$SKETCH_PONTE" "$PONTE" "$FLAGS_PONTE" "ponte" || exit 1

CONFERIR=("$PONTE")

if [[ "$SO_PONTE" == 0 ]]; then
    sleep 2
    grava robot_rx "$ROBO" "$FLAGS_ROBO -DMY_ROBOT_ID=$ROBOT_ID $COMPENSA" "robô (id $ROBOT_ID)" || exit 1
    CONFERIR+=("$ROBO")
fi

sleep 3
echo
echo "### conferindo os banners"
for d in "${CONFERIR[@]}"; do
    echo "  $d:"
    banner_de "$d" | sed 's/^/    /'
done

echo
case "$MODO" in
    feira) echo "Pronto para jogar. Para o debug_panel enxergar, use: $0 debug" ;;
    debug) echo "Pronto para o ./tools/painel.py. ANTES DA FEIRA: $0 feira" ;;
    probe) echo "Ponte medindo entrega. Para voltar ao jogo: $0 feira" ;;
esac
