#!/usr/bin/env bash
# Sobe e derruba o simulador sem deixar nó órfão.
#
# Rodar o launch em background e matar depois é traiçoeiro: `pkill -f ros2` casa
# com o próprio shell que está executando o pkill. Aqui o launch vai para um
# grupo de processos próprio (setsid) e guardamos o PGID, então o kill acerta
# exatamente a árvore certa.
#
#   ./tools/sim.sh start [args do launch...]
#   ./tools/sim.sh stop
#   ./tools/sim.sh restart difficulty:=FACIL
#   ./tools/sim.sh status

# Sem `set -u`: os setup.bash do ROS leem variáveis não definidas e abortariam.
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDFILE="$WS/.sim.pgid"
LOGFILE="$WS/.sim.log"

start() {
  if status --quiet; then
    echo "já está rodando (pgid $(cat "$PIDFILE")). use 'stop' ou 'restart'."
    return 1
  fi

  source /opt/ros/humble/setup.bash
  source "$WS/install/setup.bash"

  setsid nohup ros2 launch startup sim.py "$@" > "$LOGFILE" 2>&1 < /dev/null &
  local pid=$!

  # setsid faz o filho virar líder do próprio grupo, então PGID == PID.
  echo "$pid" > "$PIDFILE"

  for _ in $(seq 30); do
    if curl -s -o /dev/null http://localhost:8080/ 2>/dev/null; then
      echo "simulador no ar: http://localhost:8080  (pgid $pid)"
      return 0
    fi
    sleep 0.5
  done

  echo "não subiu em 15s. últimas linhas do log:"
  tail -20 "$LOGFILE"
  return 1
}

stop() {
  if [[ ! -f "$PIDFILE" ]]; then
    echo "nada para parar"
    return 0
  fi

  local pgid
  pgid="$(cat "$PIDFILE")"

  kill -TERM -"$pgid" 2>/dev/null

  for _ in $(seq 20); do
    kill -0 -"$pgid" 2>/dev/null || break
    sleep 0.25
  done

  kill -KILL -"$pgid" 2>/dev/null
  rm -f "$PIDFILE"
  echo "parado"
}

status() {
  local quiet=0
  [[ "${1:-}" == "--quiet" ]] && quiet=1

  if [[ -f "$PIDFILE" ]] && kill -0 -"$(cat "$PIDFILE")" 2>/dev/null; then
    [[ $quiet -eq 0 ]] && echo "rodando (pgid $(cat "$PIDFILE"))"
    return 0
  fi

  rm -f "$PIDFILE"
  [[ $quiet -eq 0 ]] && echo "parado"
  return 1
}

case "${1:-}" in
  start)   shift; start "$@" ;;
  stop)    stop ;;
  restart) shift; stop; sleep 1; start "$@" ;;
  status)  status ;;
  log)     tail -f "$LOGFILE" ;;
  *)       echo "uso: $0 {start|stop|restart|status|log} [args]"; exit 1 ;;
esac
