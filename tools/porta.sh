#!/usr/bin/env bash
# Diz qual porta serial é a PONTE de rádio, para você não ter que adivinhar.
#
#   ./tools/porta.sh          imprime só a porta da ponte (serve em $( ))
#   ./tools/porta.sh --tudo   tabela: cada placa plugada e o que é
#
#   ros2 launch startup teleop.py serial_port:=$(./tools/porta.sh)
#
# POR QUE ISTO EXISTE
#   Apontar o `radio_communication` para a placa errada NÃO dá erro: a porta
#   abre, os bytes saem, e o Nano do robô ignora tudo. O sintoma é o robô
#   parado com todos os logs limpos — o pior tipo. Porta inexistente reclama no
#   log; porta trocada, não.
#
#   E o número não é estável: `/dev/serial/by-id/` não distingue as duas placas
#   (as duas são CH340 sem número de série) e o ttyUSB0/1 troca a cada replug.
#   O único jeito confiável é o banner de boot — o mesmo truque do `gravar.sh`.
#
# POR QUE VARRE ttyACM TAMBÉM
#   Nano e Uno clone usam CH340 e caem em /dev/ttyUSB*; Uno oficial usa o
#   ATmega16U2, que é CDC e cai em /dev/ttyACM*. Varrendo só ttyUSB, uma ponte
#   em Uno oficial fica INVISÍVEL aqui — e o sintoma é "não achei a ponte" com a
#   placa plugada, piscando, funcionando. O banner e o reset por DTR são iguais
#   nos dois (o 16U2 reseta o 328p no DTR e não re-enumera, então o nome da
#   porta não muda depois do reset nem depois de gravar).
#
# CUSTO: ler o banner RESETA a placa (é o `hupcl` que provoca o reset). Não rode
# isto com o jogo no ar; rode antes de subir o stack.

set -u

MODO="${1:-porta}"

case "$MODO" in
    porta|--tudo) ;;
    *) sed -n '2,25p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 2 ;;
esac

banner_de() {
    local dev="$1" tmp
    tmp="$(mktemp)"
    stty -F "$dev" 115200 raw -echo hupcl 2>/dev/null || { rm -f "$tmp"; return 1; }
    timeout 7 cat "$dev" > "$tmp" 2>/dev/null
    tr -d '\r' < "$tmp"
    rm -f "$tmp"
}

PONTE=""
PONTES=()
ACHOU_ALGO=0

for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [[ -e "$dev" ]] || continue
    ACHOU_ALGO=1

    b="$(banner_de "$dev")"

    # Mesmos padrões do gravar.sh. Mudou o banner de um firmware, mude nos dois.
    case "$b" in
        *tx_bridge*|*PROBE*)  papel="ponte"
                              PONTES+=("$dev")
                              [[ -z "$PONTE" ]] && PONTE="$dev" ;;
        *robot_rx*|*sizeof*)  papel="robô" ;;
        *robot_forward*)      papel="robô (robot_forward)" ;;
        *)                    papel="não identifiquei (banner: ${b:-vazio})" ;;
    esac

    [[ "$MODO" == "--tudo" ]] && printf '%-16s %s\n' "$dev" "$papel"
done

if [[ "$ACHOU_ALGO" == 0 ]]; then
    echo "nenhum /dev/ttyUSB* nem /dev/ttyACM*. Placa desplugada — ou o brltty" >&2
    echo "roubou o CH340 (só afeta Nano e Uno clone):" >&2
    echo "  systemctl mask brltty.service brltty-udev.service" >&2
    exit 1
fi

# Duas pontes ligadas ao mesmo tempo NÃO é o dobro de alcance: as duas escrevem
# no mesmo canal e endereço, e o robô recebe as duas intercaladas — gagueira sem
# nada no log, o mesmo acidente do `use_joy` + `use_keyboard`. Escolher a
# primeira calado seria escolher no sorteio da ordem de plugar, então avisa.
# Vai para o stderr de propósito: o $( ) continua limpo.
if [[ "${#PONTES[@]}" -gt 1 ]]; then
    echo "AVISO: ${#PONTES[@]} pontes plugadas (${PONTES[*]}) — usando $PONTE." >&2
    echo "Só uma pode falar por vez. Desplugue as outras antes de subir o stack." >&2
fi

if [[ -z "$PONTE" ]]; then
    echo "não achei a ponte." >&2
    echo "Placa muda não é placa morta: um firmware que não imprime no boot fica" >&2
    echo "calado. Veja o que tem plugado com: $0 --tudo" >&2
    exit 1
fi

[[ "$MODO" == "--tudo" ]] && echo && echo "ponte: $PONTE"
[[ "$MODO" == "porta" ]] && echo "$PONTE"
exit 0
