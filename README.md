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
| `firmware/` — TX bridge e firmware do robô | pronto; salto de rádio validado em 06/08 |
| `simulator` — física do campo + GUI no navegador | pronto |
| `ai_player` — o adversário | pronto |
| `game_master` — regras, cronômetro, ranking e telas | pronto |
| `vision_game` — detecção da bola e dos robôs | detecta 100% a 30 Hz, calibrado |

## O que falta para o jogo estar 100%

**O salto de rádio foi validado em 06/08** — a ponte envia e o robô recebe.
Com isso a cadeia inteira, do olho à roda, está fechada pelo menos uma vez.
O que falta agora é montagem e aferição, não arquitetura.

1. **O robô só obedece a si mesmo com o `robot_rx.ino`.** O `franky.ino` não
   filtra por `robot_id` — ele aceita todo pacote que passe no checksum. Com um
   robô é indiferente e até cômodo; com dois, os dois andam juntos e o jogo não
   existe. Antes de montar o segundo robô, grave o `firmware/robot_rx` com
   `MY_ROBOT_ID` diferente em cada um e escreva o número no chassi.

2. **`wheel_speed_max` é um chute declarado** (0,75 m/s). É ele que converte m/s
   para a faixa −1…1 do firmware. Meça cronometrando um metro a `--left 1.0
   --right 1.0` no `tools/radio_console.py`. Errado, o robô fica lento demais ou
   satura e perde a proporção entre as rodas nas curvas.

3. **A exatidão da visão nunca foi medida contra régua**, e o **ângulo** em
   especial nunca foi conferido contra referência física — só contra leitura de
   imagem. É o erro mais caro que pode estar escondido: ângulo trocado faz o
   robô andar de ré, e nenhum teste feito até agora pegaria isso. A proporção
   dos cantos calibrados fecha em 0,10%, o que é bom sinal, mas não prova
   posição absoluta.

4. **O controle do visitante nunca foi testado com gamepad plugado.** O
   `use_joy:=true` sobe o nó e foi verificado, mas sem hardware na porta.

Comandos do dia a dia: **`CHEATSHEET.md`**. Mapa dos nós e invariantes: **`CLAUDE.md`**.

## Arquitetura

A IA não tem um caminho próprio até o robô: ela **publica um `sensor_msgs/Joy` sintético**
em `/joy_0` e desce pelo mesmo pipeline do jogador humano. Isso mantém um só caminho de
código entre a decisão e o motor — o que a IA faz é indistinguível, para o resto do
sistema, de alguém segurando um controle.

```
  câmera                                controle do jogador
     │                                          │
 vision_game                            game_controller_node
     │ /game_data                               │
     ├──────────► ai_player ──/joy_0──┐         │
     │             (robô 0)  sintético│         │ /joy_1  (robô 1)
     │                ▲               │         │
     │                │               └────┬────┘
     │          /ai/enabled            joy_aggregator
     │                │                    │ /joy_list
     │                │        ┌───────────┴───────────┐
     │                │  direction              special_controls
     │                │  /direction                 /actions
     │                │        └───────────┬───────────┘
     │                │                cinematica
     │                │                    │ /motorVelocities
     │                │            radio_communication
     │                │                    │ serial 115200
     │                │                tx_bridge ──nRF24──► robot_rx (×N)
     │                │
     └──► game_master ┘──WebSocket──► TV e operador (:8090)
```

`/ai/enabled` é a coleira: o `game_master` só libera a IA no estado `JOGANDO`.
Fora da partida ela manda zero, e `/motorVelocities` fica zerado — é o
comportamento correto, e a primeira coisa a conferir quando "a IA não faz nada".

O mapa completo de tópicos, tipos e invariantes está em `CLAUDE.md`.

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

## O jogo completo

```bash
ros2 launch startup game.py
```

Três telas:

| | onde | para quem |
|---|---|---|
| **TV** | http://localhost:8090/ | público — tela cheia no monitor |
| **Operador** | http://localhost:8090/operador | quem toca o estande |
| Simulador | http://localhost:8080/ | só enquanto não há robôs |

### O formato

Primeiro a **2 gols** vence, teto de **3 minutos**. O cronômetro corre do apito
até o gol que decide, e é ele que vai para o ranking — ordenado do menor para o
maior. Quem perde ou estoura o tempo não entra.

Duas decisões que vêm da feira, não do futebol:

- **Teto de tempo.** Fila parada é o pior inimigo de um estande. Sem teto, uma
  dupla travada em 1×1 segura vinte pessoas.
- **Só o vencedor entra no ranking.** Faz a lista significar uma coisa só —
  "quem venceu mais rápido" — em vez de misturar critérios.

Ajustáveis: `ros2 launch startup game.py target_score:=3 time_limit:=120.0`

### O operador

Digita o nome, aperta Enter. Teclas `1` (gol do jogador), `2` (gol do robô) e
`espaço` (pausa) funcionam sem precisar mirar em botão com fila esperando.

**Os botões de gol são o seguro do estande.** Estão lá desde o primeiro dia e
nunca saem: valem quando a visão não enxergar o lance, e funcionam com a
detecção automática ligada.

### Quem apita

O árbitro é o `game_master`, lendo `/game_data` — **não** o simulador. É o que
faz a regra do gol ser literalmente o mesmo código no simulador e no campo real,
já que os dois publicam a mesma mensagem. Rodando com o `game_master`, o
simulador recebe `auto_referee:=False` e passa a só obedecer `/sim/reset`; sem
ele, apita sozinho para dar para brincar.

Um gol só conta de novo depois que a bola **sai** da área. Travar por tempo não
basta: se ela ficar presa no fundo do gol — o que acontece de verdade quando
ninguém recoloca — o lockout expira e o placar dispara em série.

### Ranking

Em `~/.vss-game/highscores.json`, top 10 por menor tempo. Sobrevive a reiniciar.
Grava em arquivo temporário e move, então falta de energia no meio da escrita
não corrompe a lista. Arquivo ilegível vira aviso no log e lista vazia — nunca
derruba o jogo no meio da feira.

## Simulador — desenvolvendo sem hardware

```bash
ros2 launch startup sim.py
```

Abra **http://localhost:8080**. `WASD` dirige o robô 1, arraste a bola e os robôs
com o mouse, solte a bola em movimento para chutar.

O ponto do simulador não é conveniência, é arquitetura: ele substitui o hardware
**exatamente na fronteira dele**.

|  | com hardware | no simulador |
|---|---|---|
| entra o comando | `radio_communication` → rádio | `sim_node` aplica a física |
| sai a posição | `vision_game` → `/game_data` | `sim_node` → `/game_data` |
| entra o jogador | `game_controller_node` → `/joy_1` | teclado da GUI → `/joy_1` |

Todo o miolo — `joy_aggregator`, `direction`, `special_controls`, `cinematica` —
é o mesmo código nos dois casos. A IA escrita contra o simulador funciona nos
robôs sem alteração.

### Convenção do jogo

**Robô 0 é a IA**, defende o gol da esquerda (magenta). **Robô 1 é o visitante**,
defende o da direita (ciano).

### Testando a IA contra uma visão imperfeita

A câmera real treme e atrasa. Uma IA afinada contra posições perfeitas fica
nervosa quando encontra a câmera:

```bash
# posições com ruído de 5 mm e 80 ms de atraso
ros2 launch startup sim.py vision_noise:=0.005 vision_delay:=0.08
```

Desenvolva com os dois em zero; antes de dar a IA por pronta, ligue e veja se
ela ainda joga.

### A física

`src/simulator/simulator/physics.py` não depende de ROS — dá para importar e
testar direto. Modela o que importa para afinar a IA: inércia do motor
(`accel_tau`), atrito da bola, colisão com impulso, quique nas paredes, robôs que
não se atravessam, e o fato de que **comando de roda não é velocidade de roda**.

Não modela: derrapagem, queda de bateria, folga da transmissão. Se a IA depender
de precisão fina de posicionamento, desconfie — o robô real não tem essa precisão.

## A IA

```bash
ros2 launch startup sim.py difficulty:=DIFICIL
```

Ela **publica um `Joy` sintético** em `/joy_0`, como se fosse alguém segurando um
controle. Não fala com o motor direto. Assim ela passa pelo `joy_aggregator`,
`direction` e `cinematica` exatamente como o humano — sofre as mesmas saturações
e os mesmos limites. Mexeu na cinemática, mexeu para os dois.

### Como ela joga

Um robô diferencial não anda de lado. Se for direto na bola, empurra para onde
estiver apontando, que quase nunca é o gol. Então o alvo dela **desliza** conforme
o alinhamento:

- alinhada atrás da bola → mira 10 cm **além** dela, atravessa e empurra
- desalinhada → o alvo recua para trás da bola, e ela contorna

Isso é contínuo de propósito. Uma versão com máquina de estados
(`ATACAR`/`POSICIONAR` com limiar) travava: parada a 2 cm do alvo, com 2,7 cm de
erro lateral, ela oscilava entre os dois estados sem se mover — o alvo caía na
zona morta do controlador. O benchmark abaixo pegou isso.

### As dificuldades

Cada uma é uma limitação real de robótica, que dá para explicar apontando para o campo:

| | FÁCIL | MÉDIO | DIFÍCIL | o que é |
|---|---|---|---|---|
| `speed_frac` | 45% | 65% | 90% | fração da força que ela usa |
| `reaction_delay` | 400 ms | 250 ms | 100 ms | ela te vê no passado |
| `replan_period` | 600 ms | 400 ms | 150 ms | quantas vezes por segundo ela *pensa* |
| `home_x_max` | −0.15 | +0.10 | +0.55 | até onde vai buscar a bola |
| `aim_noise` | 6 cm | 3 cm | 1 cm | erro da visão dela |

`home_x_max` é o alcance da **busca**, não uma coleira: uma vez com a bola, ela
leva até o gol. Sem essa distinção ela empurrava até a borda da própria zona,
desistia e voltava — e nunca concluía nada.

Trocar ao vivo, sem reiniciar:

```bash
ros2 topic pub --once /ai/difficulty std_msgs/String '{data: DIFICIL}'
ros2 param set /ai_player speed_frac 0.5     # ajuste fino
ros2 topic pub --once /ai/enabled std_msgs/Bool '{data: false}'   # congela
```

Ou pelos botões na GUI.

### Medindo, em vez de adivinhar

```bash
./tools/ai_bench.py --trials 120
```

Junta o cérebro da IA com a física direto em memória — sem ROS, sem simulador — e
joga centenas de partidas em segundos. Responde "esse ajuste melhorou ou piorou?"
com número, não com trinta segundos de tela.

Contra gol livre, hoje:

```
FACIL     gol em  0.0% das tentativas
MEDIO     gol em 15.0%   | mediana 19.1s
DIFICIL   gol em 97.5%   | mediana  8.0s
```

FÁCIL em zero é o esperado — ela quase não ataca, é zagueira. Se esses números
caírem depois de uma mexida, a mexida foi ruim.

### O que aparece na GUI

A cruz é o alvo que ela escolheu. O círculo tracejado é **onde ela acha que a
bola está** — com atraso de reação ligado, ele fica visivelmente atrás da bola
real (chega a 40 cm numa bola rápida). É a explicação visual de por que ela erra,
sem precisar de palavra nenhuma. Vai para a TV na feira.

## A visão

```bash
ros2 launch startup vision.py          # só a visão, para calibrar
ros2 launch startup game.py use_vision:=true   # o jogo, com a câmera
```

Publica exatamente o mesmo `/game_data` que o simulador publicava — é por isso
que `use_vision:=true` troca uma peça só e IA, `game_master` e TV não percebem.

### Calibrando

Abra **http://localhost:8070**. Nada aqui depende de saber os números de
antemão — dá para calibrar clicando.

**Se a câmera só mudou de lugar** (campo e luz iguais), aperte
**Reencontrar cantos**. Ele alinha o quadro de agora com a foto guardada na
última calibração e transporta os cantos. É o caminho normal do dia a dia, e
funciona porque a câmera e a lâmpada são fixas — só a posição muda.

**Se for a primeira vez**, ou se o alinhamento falhar:

1. **Clicar cantos** → clique os 4 cantos do retângulo de jogo, do gol da
   esquerda/lado de cima, no sentido horário. O campo tem os cantos chanfrados
   a 45°: mire na **interseção virtual** das duas retas, não na ponta do
   chanfro.
2. **Confira pelas linhas azuis.** Elas são o campo teórico projetado pela
   homografia. Calibração certa = elas caem em cima das linhas pintadas,
   principalmente a do meio. Isso vale mais que qualquer número: erro em
   metros é difícil de julgar, linha do meio fora do lugar não é.
3. **Clicar na cor** → escolha o alvo e clique no objeto no vídeo. A faixa HSV
   sai da amostra. Os sliders ficam para ajuste fino.
4. **Salvar.** Grava `~/.vss-game/vision.json` **e** a foto de referência em
   `~/.vss-game/vision_ref.png` — é ela que faz o "Reencontrar cantos"
   funcionar da próxima vez.

O recorte da imagem sai sozinho dos cantos: calibrou, o resto vem junto.

Por que não há detecção automática de canto do zero: as bordas laterais do
campo são **interrompidas pela boca do gol**, então viram segmentos curtos,
enquanto a linha da grande área é longa e contínua. Todo ajuste de reta prefere
a área e erra o campo em ~10 cm. Duas tentativas, dois fracassos pelo mesmo
motivo — daí o alinhamento contra referência, que não depende de achar linha
nenhuma.

### As cores

Quatro cores, todas a mais de 30° de matiz umas das outras:

| papel | cor | H |
|---|---|---|
| bola | laranja | ~4 |
| retângulo do robô 0 | amarelo | ~22 |
| retângulo do robô 1 | verde | ~65 |
| quadrado de orientação (nos dois) | azul-bebê | ~98 |

Com **um robô por time**, a cor do retângulo já é a identidade e o vetor
retângulo→quadrado dá o ângulo. Não é preciso distinguir os quadrados de ID
entre si — que é justamente a parte frágil.

**Sob a lâmpada do estande, o feltro fica azulado** — H 105–114, com saturação
chegando a 109 no percentil 90. Isso encosta no azul-bebê (H 102–105, S 99–112)
em matiz *e* em saturação, e o que sobra separando é só o brilho. Por isso o
`v_min` do `front` é 105, bem mais alto que os outros. **Se trocarem a lâmpada,
é o primeiro limiar a rever**, e a saída melhor é trocar o azul-bebê por uma cor
longe do azul.

### Coisas que custaram depuração

- **`import cv2` quebra** se houver numpy 2.x em `~/.local` (o cv2 do apt é
  compilado contra numpy 1.x). Os launch files passam `PYTHONNOUSERSITE=1`;
  rodando o nó na mão, passe também.
- **O backend de captura é o `ffmpeg`, não o OpenCV**, e a diferença é grande:
  16,7 fps contra 30,1 fps a 1080p. O gargalo é o decode do MJPEG, que o
  OpenCV faz num thread só. `backend:=opencv` volta atrás se faltar ffmpeg.
- **Trave exposição, ganho, foco e balanço de branco** — o nó faz isso sozinho.
  Com os automáticos ligados, qualquer objeto claro entrando em campo reexpõe a
  câmera e derruba todos os limiares de uma vez.
- **O `exposure_time_absolute` satura no teto do frame rate** (~312 a 30 fps) e
  é quantizado pelo driver: peça 200 e leia 156 de volta. Quem controla o
  brilho é o **gain**. E o gain já foi visto voltando sozinho para 8 depois que
  o stream abre — por isso o nó relê e reaplica.

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
