# vss-game — mapa do esqueleto

Jogo de futebol de robôs para a Feira de Profissões: um visitante pega o
controle e enfrenta um robô com IA numa melhor de três, com ranking numa TV.
ROS 2 Humble, C++17 e Python.

Este arquivo é o mapa para quem chega sem contexto. O `README.md` é a narrativa
para humanos; aqui ficam os contratos, as invariantes e as armadilhas — o que é
caro de redescobrir.

---

## A ideia que organiza tudo

**A IA não tem caminho próprio até o motor.** Ela publica um `sensor_msgs/Joy`
sintético e desce pelo mesmo pipeline do jogador humano. Existe um só caminho
de código entre decisão e motor.

**O simulador e a câmera são intercambiáveis.** Os dois publicam exatamente o
mesmo `/game_data`. Trocar um pelo outro não muda nada rio abaixo — nem a IA,
nem o árbitro, nem a TV percebem.

Corolário prático: quase todo bug de "o robô faz coisa estranha" é de um lado
só dessa fronteira. Descubra de que lado antes de investigar.

---

## Topologia

```
 vision_game ──/game_data──┬─────────► game_master ──┬─/ai/enabled─────────┐
  (ou simulator)           │           (árbitro)     ├─/game/status ──► WS :8090
                           ▼                         ├─/game/highscores    │
                       ai_player ◄───────────────────┴─/ai/difficulty ─────┘
                           │
                        /joy_0                    /joy_1 ◄── jogador humano
                           └──────────┬──────────────┘
                                joy_aggregator
                                      │ /joy_list
                          ┌───────────┴───────────┐
                      direction              special_controls
                          │ /direction            │ /actions
                          └───────────┬───────────┘
                                  cinematica
                                      │ /motorVelocities
                              radio_communication
                                      │ serial 115200
                                  tx_bridge ──nRF24──► robot_rx (×N)
```

| nó | pacote | ling. | consome | produz |
|---|---|---|---|---|
| `vision_game` | `vision_game` | py | `/ai/debug` | `/game_data` |
| `simulator` | `simulator` | py | `/motorVelocities`, `/ai/debug`, `/sim/reset` | `/game_data`, `/joy_<player_id>` |
| `ai_player` | `ai_player` | py | `/game_data`, `/ai/enabled`, `/ai/difficulty` | `/joy_<ai_id>`, `/ai/debug` |
| `joy_aggregator` | `controller_interpreter` | C++ | `/joy_0`…`/joy_N` | `/joy_list` |
| `direction` | `controller_interpreter` | C++ | `/joy_list` | `/direction` |
| `special_controls` | `controller_interpreter` | C++ | `/joy_list` | `/actions` |
| `cinematica` | `cinematica` | C++ | `/direction`, `/actions` | `/motorVelocities` |
| `game_master` | `game_master` | py | `/game_data`, `/ai/difficulty` | `/game/status`, `/game/highscores`, `/ai/enabled`, `/ai/difficulty`, `/sim/reset` |
| `radio_communication` | `robot_communication` | C++ | `/motorVelocities` | serial |
| `keyboard_input` | `controller_interpreter` | C++ | teclado | `/joy_0` |

Tipos: `/game_data` `GameData` · `/joy_N` `sensor_msgs/Joy` · `/joy_list`
`JoyList` · `/direction` `DirectionList` · `/actions` `ActionsList` ·
`/motorVelocities` `MotorVelocitiesList` · `/ai/debug` `AiDebug` ·
`/ai/enabled` `std_msgs/Bool` · `/ai/difficulty` `std_msgs/String` ·
`/game/status` `GameStatus` · `/game/highscores` `HighScoreList` ·
`/sim/reset` `std_msgs/Empty`. Tudo sem prefixo de pacote é
`shared_interfaces/msg`.

O controle do jogador físico vem do `game_controller_node`, do pacote **`joy`
do ROS** (não deste repo), remapeado `/joy` → `/joy_N`.

### Fios que o desenho não mostra

- **`/ai/enabled` é a coleira da IA.** O `game_master` publica `True` apenas
  quando o estado é `JOGANDO` (`master_node.py:148`). Fora da partida a IA
  manda zero e o `/motorVelocities` fica zerado — **isso é correto, não é bug**.
  É a primeira coisa a checar quando "a IA não faz nada".
- **O `game_master` assina o próprio `/ai/difficulty`** que publica, para que um
  `ros2 topic pub` externo mude a dificuldade e a TV acompanhe.
- **As GUIs não são ROS**, são WebSocket/HTTP: `:8090` TV e operador
  (`game_master`), `:8080` simulador, `:8070` calibração da visão.
- **`/ai/debug` existe para a TV**, não para depurar. É o conteúdo pedagógico:
  mostra ao público o alvo escolhido e onde a IA *acha* que a bola está.

---

## Convenções que o código inteiro assume

- **Robô 0 é a IA**, defende o gol da **esquerda**. **Robô 1 é o visitante**,
  defende o da **direita**. Mudar isso quebra a IA, a TV e o simulador juntos.
- **Coordenadas**: metros, origem no **centro** do campo, `x` ao longo do
  comprimento (1,50 m), `y` ao longo da largura (1,30 m), `orientation` em
  radianos com 0 = +x e crescendo anti-horário.
- Dimensões oficiais em `simulator/physics.py:FieldSpec`. É a fonte da verdade;
  o `vision_game` e o `game_master` recebem por parâmetro de launch.

---

## Contratos que não podem quebrar

**1. `GameData` é a fronteira do hardware.** Quem publica pode ser o simulador
ou a câmera. Consumidores tratam ausência assim:

- `ball_detected=false` → a IA para (`ai_node.py:210`) e o `game_master` ignora
  o frame (`master_node.py:116`).
- robô ausente do array → a IA para também. **Nunca invente posição** para
  preencher; o comportamento de "não vi" já está definido rio abaixo.

**2. O pacote do rádio tem 14 bytes e vive em três lugares.** Mudou um, mude os
três: `shared_interfaces/include/shared_interfaces/RadioMessage.h`,
`firmware/tx_bridge/tx_bridge.ino`, `firmware/robot_rx/robot_rx.ino`. Há um
`static_assert` de 14 bytes, e `tools/radio_test.py --dry-run` monta o mesmo
pacote em Python para comparar os dois lados.

**3. Cada robô precisa de `MY_ROBOT_ID` diferente** em `robot_rx.ino`. Dois
robôs com o mesmo ID andam juntos.

---

## Rodando

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch startup game.py                    # jogo no simulador
ros2 launch startup game.py use_vision:=true   # jogo com a câmera
ros2 launch startup vision.py                  # só a visão, para calibrar
ros2 launch startup sim.py                     # simulador solto
ros2 launch startup teleop.py num_robots:=2    # teleoperação pura

./tools/stop.sh            # derruba tudo (use isto, ver abaixo)
./tools/stop.sh --check    # o que está rodando e quais portas
./tools/ai_bench.py        # mede a IA sem ROS, centenas de partidas em segundos
./tools/radio_test.py      # fala com a ponte sem ROS (linha de comando)
./tools/radio_console.py   # o mesmo, com teclado no navegador (:8060)
                           # + taxa de entrega, se a ponte tiver o tx_probe
```

Gravar firmware (`arduino-cli` em `~/.local/bin`):

```bash
export PATH="$HOME/.local/bin:$PATH"
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 firmware/tx_bridge
arduino-cli upload -p /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
                   --fqbn arduino:avr:nano:cpu=atmega328 firmware/tx_bridge
```

**Use sempre o caminho `by-id` da ponte.** Só ela tem número de série, então
`/dev/ttyUSB0` pode virar o Arduino errado quando há dois plugados. No Pop!_OS,
se o `/dev/ttyUSB*` não aparecer, é o `brltty` roubando o CH340 — já foi
resolvido com `systemctl mask brltty.service brltty-udev.service`.

**Sempre pare com `./tools/stop.sh`.** Matar por nome de processo deixa metade
do stack vivo, porque `cinematica`, `direction`, `joy_aggregator` e
`special_controls` são C++ e vivem em caminhos diferentes dos nós Python. Um
`simulator` órfão publicando `/game_data` junto com a câmera dá 74 Hz numa
câmera de 30 e posições alternando entre duas fontes — sintoma que não aponta
para a causa.

---

## Armadilhas do ambiente (medidas, não supostas)

**`import cv2` quebra se houver numpy 2.x em `~/.local`.** O OpenCV do apt é
compilado contra numpy 1.x. A saída é `PYTHONNOUSERSITE=1`, que ignora o
user-site sem mexer nos pacotes do usuário — os launch files já passam isso via
`additional_env`. Rodando o nó na mão, passe também.

**A captura usa `ffmpeg` por pipe, não `cv2.VideoCapture`.** Medido a 1080p
MJPG nesta câmera: OpenCV 16,7 fps, ffmpeg 30,1 fps. O gargalo é o decode do
MJPEG, que o OpenCV faz num thread só. `backend:=opencv` volta atrás.

**Controles v4l2 têm ordem obrigatória:**
1. Só depois que o **streaming começa** — o teto do `exposure_time_absolute`
   depende do intervalo de frame negociado na abertura. Configurar antes trava
   o exposure na metade e derruba todas as cores **sem nenhum erro no log**.
2. **Um a um**, nunca em lote: `focus_absolute` fica inativo enquanto o
   autofoco está ligado, e o driver rejeita o lote inteiro com
   `Permission denied`.
3. **Reconferir depois** com `--get-ctrl`: o `gain` já foi visto voltando
   sozinho para 8 depois que o stream abre.

**O exposure satura no teto do frame rate** (~312 a 30 fps) e é quantizado pelo
driver — peça 200 e leia 156 de volta. Quem controla o brilho é o **gain**.

---

## Visão — o que é específico

Câmera Logitech C920 em `/dev/video2`, 1920×1080 MJPG a 30 Hz, com luminária
dedicada sobre o campo. Calibração em `~/.vss-game/vision.json` mais a foto de
referência `~/.vss-game/vision_ref.png`.

**Fluxo normal:** a câmera e a lâmpada são fixas; só a **posição** da câmera
muda entre montagens. Então o botão **Reencontrar cantos** na GUI alinha o
quadro de agora contra a foto de referência (ORB + RANSAC) e transporta os
cantos. Só se isso falhar é que se clica de novo.

**A conferência é visual**: a GUI projeta as marcações do campo pela homografia.
Calibração certa = as linhas caem sobre as pintadas, e a de meio-campo é a mais
fácil de julgar. Vale mais que erro em metros, que ninguém sabe julgar.

**Não existe detecção automática de canto do zero, e não é por falta de
tentativa.** Duas abordagens falharam pelo mesmo motivo físico: as bordas
laterais do campo são **interrompidas pela boca do gol**, viram segmentos
curtos, e todo ajuste de reta prefere a linha da grande área — errando o campo
em ~10 cm. Procurar quadrilátero no contorno também não serve: o campo tem os
cantos chanfrados a 45°, o contorno é um octógono.

**Etiquetas** (um robô por time, então a cor do retângulo já é a identidade e o
vetor retângulo→quadrado dá o ângulo):

| papel | cor | H |
|---|---|---|
| bola | laranja | ~7 |
| retângulo robô 0 | amarelo | ~25 |
| retângulo robô 1 | verde | ~66 |
| quadrado de orientação (nos dois) | azul-bebê | ~98–105 |

Armadilhas de cor, todas medidas:

- **`team_a` (amarelo) precisa de `s_min` baixo.** Subir de 55 para 85 derruba
  o blob de 1033 px para 186 px.
- **Sob esta lâmpada o feltro fica azulado** (H 105–114, S até 109 no p90), o
  que invade o azul-bebê em matiz *e* saturação. Só o brilho separa, daí
  `v_min=105` no `front`. **Trocou a lâmpada, revise este limiar primeiro** — e
  a saída melhor é trocar o azul-bebê por uma cor longe do azul.
- **Azul-escuro não serve como cor de time.** Fica a 3° do azul-bebê, e a mesma
  cor medida em dois pontos do campo varia 5°. Foi por isso que virou verde.
- Nada além da bola deve ser laranja/vermelho vivo. Há uma guarda geométrica
  (candidato colado num robô é descartado) e uma em metros (fora do campo é
  descartado), mas a margem é melhor que a guarda.

---

## Estado

| módulo | situação |
|---|---|
| `shared_interfaces`, `controller_interpreter`, `cinematica`, `startup` | prontos |
| `simulator`, `ai_player`, `game_master` | prontos |
| `vision_game` | detecta 100% a 30 Hz, calibrado |
| `robot_communication` + `firmware/` | prontos; **rádio validado em 06/08** (ponte envia, robô recebe) |

Calibração salva em `~/.vss-game/vision.json` — proporção dos cantos fechando
em 0,10% do campo real.

**Falta para o jogo estar 100%** — ver `README.md` para o detalhe. A cadeia
do olho à roda já fechou pelo menos uma vez; o que resta é montagem e aferição:

1. **o `franky.ino` não filtra por `robot_id`** — obedece todo pacote. Serve
   para um robô na bancada, não serve para dois em campo. Antes do segundo
   robô, gravar `firmware/robot_rx` com `MY_ROBOT_ID` distinto em cada um
2. `wheel_speed_max` (0,75) é chute declarado — medir cronometrando um metro
3. exatidão da visão nunca medida contra régua; o **ângulo** em especial nunca
   foi conferido contra referência física
4. `use_joy` nunca testado com gamepad plugado
