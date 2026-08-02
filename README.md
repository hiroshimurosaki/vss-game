# vss-game

Jogo do **Carrossel Caipira** para a Feira de Profissões: um visitante pega o controle
e enfrenta um robô com IA numa melhor de três. Quem vence entra num ranking dos dez
melhores tempos, exibido numa TV.

Feito sobre a base do [VSS](https://github.com/carrossel-caipira/VSS) (ROS 2 Humble, C++17)
e do [VSS_Arduino](https://github.com/carrossel-caipira/VSS_Arduino).

## Estado

| Módulo | Status |
|---|---|
| `shared_interfaces` — mensagens ROS + protocolo do rádio | pronto |
| `controller_interpreter` — controles e teclado | pronto |
| `cinematica` — cinemática diferencial | pronto |
| `robot_communication` — ponte ROS ↔ serial | pronto |
| `startup` — launch files | pronto |
| `firmware/` — TX bridge e firmware do robô | pronto, falta validar no hardware |
| `vision_game` — detecção da bola e dos robôs | a fazer |
| `ai_player` — o adversário | a fazer |
| `game_master` — regras, cronômetro, placar | a fazer |
| `scoreboard` — a tela da TV | a fazer |

## Arquitetura

A IA não tem um caminho próprio até o robô: ela **publica um `sensor_msgs/Joy` sintético**
em `/joy_1` e desce pelo mesmo pipeline do jogador humano. Isso mantém um só caminho de
código entre a decisão e o motor — o que a IA faz é indistinguível, para o resto do
sistema, de alguém segurando um controle.

```
  câmera                    controle do jogador
     │                              │
 vision_game                 game_controller_node
     │ /game_data                   │ /joy_0
     ├──────────► ai_player ────────┤ /joy_1  (Joy sintético)
     │                              │
     │                       joy_aggregator
     │                              │ /joy_list
     │                  ┌───────────┴───────────┐
     │            direction              special_controls
     │            /direction                 /actions
     │                  └───────────┬───────────┐
     │                          cinematica
     │                              │ /motorVelocities
     │                       radio_communication
     │                              │ serial 115200
     │                          tx_bridge ──nRF24──► robot_rx (×N)
     │
 game_master ──WebSocket──► scoreboard (Chrome fullscreen na TV)
```

## Protocolo do rádio

Pacote de **14 bytes**, idêntico em três lugares — se mudar um, mude os três:

- `src/shared_interfaces/include/shared_interfaces/RadioMessage.h`
- `firmware/tx_bridge/tx_bridge.ino`
- `firmware/robot_rx/robot_rx.ino`

| campo | tipo | bytes |
|---|---|---|
| `startByte` | `uint8` | 1 (`0x14`) |
| `velMotor1` | `float` | 4 (roda esquerda, −1.0 a 1.0) |
| `velMotor2` | `float` | 4 (roda direita, −1.0 a 1.0) |
| `robotId` | `int32` | 4 |
| `checksum` | `uint8` | 1 (XOR de `velMotor1`…`robotId`) |

O `RadioMessage.h` tem um `static_assert` de 14 bytes, e `tools/radio_test.py` monta o
mesmo pacote em Python — dá para comparar os dois lados com `--dry-run`.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Dependências: `ros-humble-joy`, `libsdl2-dev`.

## Gravando o firmware

**Cada robô precisa de um `MY_ROBOT_ID` diferente.** Em `firmware/robot_rx/robot_rx.ino`:

```cpp
#define MY_ROBOT_ID 0   // 0, 1, 2, 3 — um por robô
```

Grave e **escreva o número no chassi**. Dois robôs com o mesmo ID andam juntos e o jogo
não funciona. O `tx_bridge` é único, vai no Arduino que fica ligado no notebook.

## Rodando

```bash
# teleoperação: cada controle dirige um robô
ros2 launch startup teleop.py num_robots:=2

# sem controle, usando o teclado (WASD, robô 0)
ros2 launch startup teleop.py num_robots:=1 use_keyboard:=true

# com logs detalhados
ros2 launch startup teleop.py num_robots:=2 verbose:=true
```

## Testando o rádio sem o ROS

Quando o robô não anda, a primeira pergunta é "é o robô ou é o ROS?". Esta ferramenta
responde em trinta segundos, falando direto com o `tx_bridge`:

```bash
# descobre qual chassi tem qual ID: gira um de cada vez
./tools/radio_test.py --sweep

# um robô específico, para frente
./tools/radio_test.py --id 0 --left 0.5 --right 0.5 --duration 2

# dirige pelo teclado
./tools/radio_test.py --id 0 --interactive

# só mostra os bytes, sem abrir a serial
./tools/radio_test.py --id 1 --left 0.5 --right -0.25 --dry-run
```

Não precisa de `pyserial` — usa `termios` da stdlib.

## Calibração

`wheel_speed_max` (no `cinematica`) é a velocidade de roda, em m/s, que corresponde a
PWM 100%. É o que converte m/s para a faixa −1…1 do firmware. Para medir: solte o robô a
`--left 1.0 --right 1.0` e cronometre um metro. O default de `0.75` é um chute.

Se estiver errado, o robô ou fica lento demais (valor alto demais) ou satura e perde a
proporção entre as rodas nas curvas (valor baixo demais).
