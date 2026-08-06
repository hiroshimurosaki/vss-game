#!/usr/bin/env bash
# Derruba tudo que este workspace subiu.
#
# Existe porque matar por nome de processo não funciona aqui e já custou caro:
# `cinematica`, `direction`, `joy_aggregator` e `special_controls` são C++ e
# vivem em caminhos diferentes dos nós Python, então um `pkill -f vision_node`
# deixa metade do stack viva. Nós órfãos que sobrevivem são péssimos de
# diagnosticar: um `simulator` esquecido continua publicando `/game_data` em
# paralelo com a câmera, e aí `ros2 topic hz` mostra 74 Hz numa câmera de 30 e
# as posições ficam alternando entre duas fontes.
#
# O critério aqui é o caminho de instalação do workspace, que pega todos.
#
#   ./tools/stop.sh          derruba e lista o que sobrou
#   ./tools/stop.sh --check  só mostra o que está rodando

set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

listar() {
    pgrep -af "$WS/install" 2>/dev/null | grep -v 'stop\.sh' || true
}

if [[ "${1:-}" == "--check" ]]; then
    echo "Nós deste workspace rodando agora:"
    listar | sed 's/^/  /' || true
    [[ -z "$(listar)" ]] && echo "  (nenhum)"
    echo
    echo "Portas:"
    ss -ltn 2>/dev/null | grep -E ':(8070|8080|8090)\b' | sed 's/^/  /' \
        || echo "  (8070/8080/8090 livres)"
    exit 0
fi

pids="$(pgrep -f "$WS/install" 2>/dev/null | grep -v $$ || true)"
# O `ros2 launch` supervisiona os nós e os ressuscita; morre primeiro.
lpids="$(pgrep -f 'bin/ros2 launch' 2>/dev/null || true)"

if [[ -z "$pids$lpids" ]]; then
    echo "Nada rodando."
else
    for p in $lpids $pids; do kill "$p" 2>/dev/null; done
    sleep 2
    for p in $lpids $pids; do kill -9 "$p" 2>/dev/null; done
    sleep 1
fi

# O ffmpeg da captura é filho do vision_node e costuma ir junto, mas se o pai
# morreu de SIGKILL ele fica segurando a câmera — e aí o próximo start falha
# com "Device or resource busy", que não parece ter nada a ver.
for p in $(pgrep -f 'ffmpeg .*-i /dev/video' 2>/dev/null || true); do
    kill -9 "$p" 2>/dev/null
done

sobrou="$(listar)"
if [[ -n "$sobrou" ]]; then
    echo "AINDA VIVO:"
    echo "$sobrou" | sed 's/^/  /'
    exit 1
fi
echo "Tudo parado. Câmera e portas livres."
