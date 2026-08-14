# Colinha

Todo comando assume o ambiente carregado. **Faça isto uma vez por terminal:**

```bash
cd ~/dev/vss-game
source /opt/ros/humble/setup.bash && source install/setup.bash
```

Dica: coloque um alias no `~/.bashrc` para não digitar isso na feira.

```bash
alias vss='cd ~/dev/vss-game && source /opt/ros/humble/setup.bash && source install/setup.bash'
```

---

## 1 · No dia da feira

```bash
# o jogo, com a câmera e os robôs
ros2 launch startup game.py use_vision:=true use_radio:=true use_joy:=true

# sem robôs ainda (roda tudo, mas nada se move em campo)
ros2 launch startup game.py use_vision:=true

# sem câmera nem robôs — o jogo inteiro no simulador
ros2 launch startup game.py
```

| tela | endereço | para quem |
|---|---|---|
| **TV** | http://localhost:8090/ | público, tela cheia |
| **Operador** | http://localhost:8090/operador | quem toca o estande |
| Visão + IA ao vivo | http://localhost:8070/ | só para conferir |
| Simulador | http://localhost:8080/ | só sem câmera |

No operador: digita o nome e Enter. Teclas `1` gol do jogador, `2` gol do robô,
`espaço` pausa.

Ajustes comuns:

```bash
ros2 launch startup game.py use_vision:=true target_score:=3 time_limit:=120.0
ros2 launch startup game.py use_vision:=true difficulty:=DIFICIL
```

### Se só um robô estiver funcionando

Modo duelo: um robô, dois motoristas, turnos alternados. O visitante joga o
turno dele, o Franky joga o dele **no mesmo robô**, e ganha o round quem levou
menos tempo até o gol. Melhor de três.

```bash
# na feira, com a câmera e o robô
ros2 launch startup duelo.py use_vision:=true use_radio:=true use_joy:=true \
    serial_port:=$(./tools/porta.sh)

# ensaio completo no simulador, sem robô e sem gamepad (teclado da GUI dirige)
ros2 launch startup duelo.py
```

O robô é o **id 0**, etiqueta **amarela**: `./tools/gravar.sh feira --id 0`.
Os dois motoristas atacam o **gol da direita**.

A única tarefa manual é **repor a bola no centro** entre os turnos — o robô
volta à marca sozinho, dirigido pela IA. A contagem só começa quando o árbitro
vê robô e bola no lugar (com teto de 20 s, para a fila nunca travar).

```bash
ros2 launch startup duelo.py use_vision:=true difficulty:=FACIL
ros2 launch startup duelo.py use_vision:=true turn_limit:=40.0 rounds_to_win:=3
./tools/duelo_bench.py --franky     # quanto o Franky leva, por dificuldade
```

Tempo do Franky até o gol, medido: **FÁCIL 19 s · MÉDIO 11 s · DIFÍCIL 6 s**
(medianas). Escolha o preset que deixa ele um pouco acima da média das
primeiras pessoas — e meça de novo se trocar de campo.

**Parar tudo:**

```bash
./tools/stop.sh
```

Use sempre isto, nunca `Ctrl-C` sozinho ou `pkill`. Metade dos nós é C++ e
sobrevive a mata-por-nome; um `simulator` órfão publicando `/game_data` junto
com a câmera dá diagnóstico falso.

---

## 2 · Calibrar a visão

```bash
ros2 launch startup vision.py
# abra http://localhost:8070
```

1. **Reencontrar cantos** — se só a posição da câmera mudou. Alinha contra a
   foto da última calibração.
2. Se falhar: **Clicar cantos** → 4 cliques, gol da esquerda/lado de cima
   primeiro, sentido horário. Mire na linha branca **pintada**, não na borda da
   madeira, e na interseção virtual das retas (os cantos são chanfrados).
3. **Confira pelas linhas azuis** — é o campo teórico projetado. Certo = elas
   caem sobre as pintadas. A do meio é a mais fácil de julgar.
4. **Clicar na cor** → escolha o alvo, clique no objeto no vídeo.
5. **Salvar** → grava `~/.vss-game/vision.json` + a foto de referência.

Conferir sem GUI:

```bash
ros2 topic hz /game_data           # deve dar ~30 Hz
ros2 topic echo /game_data --once  # posições em metros
```

Outra câmera ou backend:

```bash
ros2 launch startup vision.py device:=/dev/video0 backend:=opencv
v4l2-ctl --list-devices             # descobrir qual /dev/video é a C920
```

---

## 3 · Testar o rádio e os robôs (sem ROS)

```bash
# console no navegador, dirige pelo teclado
./tools/radio_console.py
# abre http://localhost:8060 — WASD dirige, soltar para, espaço freia
```

Linha de comando:

```bash
./tools/radio_test.py --sweep                              # descobre qual chassi tem qual ID
./tools/radio_test.py --id 0 --left 0.5 --right 0.5 --duration 2
./tools/radio_test.py --id 0 --interactive                 # teclado no terminal
./tools/radio_test.py --id 1 --left 0.5 --right -0.25 --dry-run   # só mostra os bytes
```

**"O robô recebeu?"** — grave a ponte de diagnóstico e olhe a porcentagem de
entrega no console:

```bash
export PATH="$HOME/.local/bin:$PATH"
arduino-cli upload -p /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --fqbn arduino:avr:nano:cpu=atmega328 firmware/tx_probe
./tools/radio_console.py
```

| o que aparece | conclusão |
|---|---|
| `PROBE RADIO FAIL` | o rádio **da ponte** não responde ao SPI: fio ou alimentação, aqui |
| `RADIO OK` + entrega 0% | ponte viva, robô não responde: robô desligado, rádio dele morto, canal/endereço |
| entrega alta + robô parado | **rádio ok.** O problema é motor, bateria, ponte H ou `MY_ROBOT_ID` |

---

## 4 · Gravar firmware

```bash
export PATH="$HOME/.local/bin:$PATH"
FQBN=arduino:avr:nano:cpu=atmega328
PONTE=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0

arduino-cli compile --fqbn $FQBN firmware/tx_bridge          # só compila
arduino-cli upload -p $PONTE --fqbn $FQBN firmware/tx_bridge # grava
```

| sketch | onde vai | para quê |
|---|---|---|
| `firmware/tx_bridge` | ponte (PC) | **o da feira** — broadcast, sem auto-ACK |
| `firmware/tx_probe` | ponte (PC) | diagnóstico — reporta entrega, um robô só |
| `firmware/robot_rx` | robô | receptor deste repo |

**Use sempre o caminho `by-id` da ponte.** Só ela tem número de série; com dois
Arduinos plugados, `/dev/ttyUSB0` pode ser o outro.

**Cada robô precisa de `MY_ROBOT_ID` diferente** em `robot_rx.ino`. Grave e
escreva o número no chassi.

Descobrir portas:

```bash
arduino-cli board list
ls -l /dev/serial/by-id/
```

---

## 5 · Desenvolver sem hardware

```bash
ros2 launch startup sim.py                 # simulador, http://localhost:8080
ros2 launch startup sim.py difficulty:=DIFICIL
ros2 launch startup sim.py vision_noise:=0.005 vision_delay:=0.08  # câmera imperfeita

./tools/ai_bench.py --trials 120           # mede a IA sem ROS nem simulador
```

Mexer na IA ao vivo:

```bash
ros2 topic pub --once /ai/difficulty std_msgs/String '{data: DIFICIL}'
ros2 topic pub --once /ai/enabled std_msgs/Bool '{data: false}'   # congela
ros2 param set /ai_player speed_frac 0.5
```

Teleoperação pura, sem jogo:

```bash
ros2 launch startup teleop.py num_robots:=2
ros2 launch startup teleop.py num_robots:=1 use_keyboard:=true    # WASD
```

---

## 6 · Build

```bash
colcon build --symlink-install                          # tudo
colcon build --symlink-install --packages-select vision_game   # um pacote
source install/setup.bash                               # depois de todo build
```

---

## 7 · Quando algo não funciona

```bash
./tools/stop.sh --check      # o que está rodando e quais portas
ros2 node list               # nós vivos
ros2 topic info /game_data   # DEVE dizer "Publisher count: 1"
ros2 topic hz /game_data     # ~30 Hz com câmera, ~60 com simulador
```

| sintoma | causa provável |
|---|---|
| `package 'startup' not found` | faltou `source install/setup.bash` |
| taxa muito acima de 30 Hz, posições pulando | dois publishers em `/game_data` → `./tools/stop.sh` |
| `address already in use` | instância anterior viva → `./tools/stop.sh` |
| a IA não faz nada, motores zerados | normal fora de `JOGANDO`: o `game_master` só libera com `/ai/enabled` |
| nada detectado, sem erro no log | exposição travou baixa; veja a linha `exposure em ...` no boot |
| `Device or resource busy` na câmera | `ffmpeg` órfão segurando → `./tools/stop.sh` |
| `/dev/ttyUSB*` não aparece | `brltty` roubando o CH340 → já resolvido com `systemctl mask brltty.service brltty-udev.service` |
| `import cv2` quebra fora do launch | falta `PYTHONNOUSERSITE=1` |

Onde ficam as coisas:

```
~/.vss-game/vision.json       calibração da visão
~/.vss-game/vision_ref.png    foto de referência (reencontrar cantos)
~/.vss-game/highscores.json   ranking
```

Recomeçar a calibração do zero: apague os dois primeiros e calibre de novo.
