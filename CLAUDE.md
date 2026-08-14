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

**2. Os gatilhos do `Joy` são "sdl" em todo lugar: solto = 0,0, fundo = −1,0.**
`axes[4]` é L2 (ré) e `axes[5]` é R2 (frente). Quem publica `Joy` publica assim
— a IA (`brain.py:to_joy_axes`), o simulador (`sim_node.py:_publish_joy`), o
`keyboard_input` — porque é a convenção que o hardware fala. Medido em
13/08/2026 com um DualShock 4, apertando até o fim:

| | L2 | R2 |
|---|---|---|
| repouso | 0,00 | 0,00 |
| fundo | **−1,00** | **−1,00** |

O sinal negativo é do próprio `game_controller_node`, que nega os eixos do SDL.

**Não meça isso com o controle dormindo.** No cabo USB o DualShock 4 carrega
sem entrar em modo de entrada: o `js0` existe, o SDL abre, o nó publica por
`autorepeat`, e **todos os eixos saem 0,0** — que é indistinguível de "solto"
se você não souber. Foi assim que a convenção foi lida errado uma vez. Aperte o
botão PS até a barra de luz acender e confira que `/dev/input/js0` emite
eventos antes de concluir qualquer coisa.

Antes disso os produtores internos usavam "signed" (solto = +1) e só o controle
físico usava a convenção real. Como `trigger_mode` é **um parâmetro só** do nó
`direction`, valendo para todos os robôs do `/joy_list`, era impossível acertar
a IA e o jogador humano ao mesmo tempo — com `use_joy:=true` um dos dois sempre
saía errado. O `DirectionNode` ainda aceita `unit` (fundo = +1) e `signed`
(solto = +1, o `joy_node` clássico), mas trocar exige mudar **todos** os
produtores juntos.

O sintoma de errar isto não aponta para a causa: em `unit`, o `clamp` joga o
−1,0 do fundo para 0,0, os dois gatilhos lêem zero e o robô fica **imóvel**
enquanto o volante continua funcionando perfeitamente — "gira mas não anda".

O ganho colateral da convenção é que o neutro é o próprio zero do array, então
ninguém esquece de preenchê-lo — e o watchdog do `joy_aggregator` zera um
controle perdido sem saber de convenção nenhuma. O espelho do `DirectionNode`
em `tools/ai_bench.py:_direction_node` precisa mudar junto; o `ai_bench` é o
teste de regressão, e a conversão não mexeu em nenhum número.

**3. O pacote do rádio tem 14 bytes e vive em três lugares.** Mudou um, mude os
três: `shared_interfaces/include/shared_interfaces/RadioMessage.h`,
`firmware/tx_bridge/tx_bridge.ino`, `firmware/robot_rx/robot_rx.ino`. Há um
`static_assert` de 14 bytes, e `tools/radio_test.py --dry-run` monta o mesmo
pacote em Python para comparar os dois lados.

**4. Cada robô precisa de `MY_ROBOT_ID` diferente**, e ele escolhe também o
**endereço de rádio** do robô (ver contrato 5). Não edite o `#define`: use
`./tools/gravar.sh debug --id 0`, que sobrescreve pelo compilador — assim os
dois robôs saem do mesmo fonte e não dá para esquecer de trocar entre uma
gravação e outra. Dois robôs com o mesmo ID andam juntos.

**5. O auto-ACK do rádio é obrigatório nos dois lados, e cada robô tem seu
endereço.** Medido em 10/08/2026, 120 pacotes a 30 Hz, robô a 1 m da ponte:

| ponte | robô | entrega |
|---|---|---|
| `setAutoAck(false)` | `false` | **0,0%** |
| `setAutoAck(false)` | `true` | **0,0%** |
| `setAutoAck(true)` | `false` | **0,0%** |
| `setAutoAck(true)` | `true` | **100,0%** |

Zero, não "ruim" — e sem erro em nenhum dos dois logs, porque `write()` sem ACK
devolve `true` assim que o pacote sai do FIFO, sem prova de que alguém ouviu. O
sintoma é robô parado com tudo limpo, que é o pior tipo. **Se o robô não anda,
confira isto antes de suspeitar de qualquer outra coisa.**

O preço do ACK é que ele só funciona com **um receptor por endereço**: dois
robôs no mesmo endereço confirmam juntos e as respostas colidem. Daí a tabela
`ENDERECOS` (`{"VSS00", "VSS01"}`, indexada por `robot_id`), que vive em **três
lugares** e precisa ser a mesma nos três: `tx_bridge.ino`, `robot_rx.ino` e
`tx_probe.ino`. A ponte troca o `openWritingPipe` conforme o `robot_id` do
pacote. `setRetries(5, 5)` é o que leva 87,5% a 100%; o default de 15 travaria o
loop por ~60 ms sempre que um robô estivesse fora do ar.

---

## Modo duelo — a contingência de um robô só

`ros2 launch startup duelo.py`. Existe para o cenário em que só um robô está de
pé no dia. O duelo simultâneo não acontece com um robô; o alternado acontece:
o visitante joga o turno dele, o Franky joga o dele **no mesmo robô**, e ganha
o round quem chegou ao gol em menos tempo. Melhor de três.

O adversário sai do campo e vai para o relógio. Ganha-se um efeito que o
formato de dois robôs não dá: **mesmo corpo, dois motoristas**, com a variável
"qual robô anda melhor" eliminada da comparação.

Peças novas: `game_master/duelo.py` (regras puras, mesmo estilo do `rules.py`),
`game_master/turn_mux.py`, `web/duelo.html`, `startup/launch/duelo.py`. O
`master_node` ganhou o parâmetro `mode` e o modo clássico não mudou de
comportamento em nada.

**1. Um robô, e ele é o 0.** A convenção não muda — robô 0 é o da IA — o
visitante é que toma o volante emprestado. Etiqueta amarela, `gravar.sh feira
--id 0`, `num_robots:=1` no agregador para a ponte não gastar airtime tentando
falar com um robô 1 que não existe. **Os dois motoristas atacam o gol da
direita**, que é o que o `brain` já ataca — se cada um atacasse um lado,
qualquer assimetria do campo entraria direto na comparação de tempos.

**2. Ninguém publica direto no `/joy_0`.** As duas fontes vão para tópicos
privados (`/duelo/joy_humano`, `/duelo/joy_ia`) e o `turn_mux` é o **único**
publicador do tópico que o robô ouve; ele decide pelo `/game/joy_source` que o
árbitro publica. Calar o que não está na vez não resolveria: o
`game_controller_node` publica por autorepeat e a IA publica zeros quando
congelada, e a intercalação dos dois é o robô gaguejando sem nada no log — o
mesmo acidente que o `use_joy` + `use_keyboard` já causou.

**3. O árbitro só solta a contagem quando VÊ o campo pronto** — robô na marca e
bola no centro, medidos do `/game_data` (`_ready_from`). Um turno que começa com
a bola adiantada não é comparável com nenhum outro. O teto `prep_max` (20 s)
existe porque a fila é mais importante que a pureza: estourou, começa assim
mesmo. Entre os turnos a IA leva o robô de volta sozinha (`/ai/home`), então a
única tarefa manual é repor a bola.

**4. Os presets de dificuldade NÃO atravessam para o duelo, e isso está
medido.** `PRESETS_DUELO` em `brain.py` existe porque, no cenário do duelo, o
`FACIL` do jogo **nunca conclui** (0% em 45 s) e o `MEDIO` conclui 82% com
mediana de 19,8 s. Duas causas: sem adversário em campo não há o que defender —
e `home_x_max` baixo faz a IA largar a bola e voltar ao gol —; e o erro de mira
se acumula sem ninguém para corrigir, porque a IA é a única coisa agindo sobre
a bola. Nos presets do duelo o ruído é baixo e fixo e **a velocidade é o único
botão**: FÁCIL 19 s · MÉDIO 11 s · DIFÍCIL 6 s de mediana, ≥95% de conclusão.
`./tools/duelo_bench.py --franky --comparar` mostra os dois lado a lado.

O `turn_limit` (30 s) precisa ficar acima do pior caso do preset em uso, senão
o próprio Franky estoura. Sobra folga no MÉDIO e no DIFÍCIL; no FÁCIL ele
estoura ~1 turno em 10, de propósito — estourar entrega o round ao visitante
justamente no ajuste mais fácil.

Ressalva honesta: o bench não simula o `reaction_delay` (ele mora no nó, que
segura os snapshots), então no campo os tempos saem um pouco piores que a
tabela. Afine ao vivo com `ros2 param set /ai_player speed_frac 0.5` — e as
sobrescritas agora **sobrevivem** à troca de dificuldade pela tela do operador,
que antes as descartava calada.

---

## Rodando

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch startup game.py                    # jogo no simulador
ros2 launch startup game.py use_vision:=true   # jogo com a câmera
ros2 launch startup duelo.py                   # UM robô só, turnos alternados
ros2 launch startup vision.py                  # só a visão, para calibrar
ros2 launch startup sim.py                     # simulador solto
ros2 launch startup teleop.py num_robots:=2    # teleoperação pura

./tools/stop.sh            # derruba tudo (use isto, ver abaixo)
./tools/stop.sh --check    # o que está rodando e quais portas
./tools/porta.sh           # qual ttyUSB é a ponte (reseta as placas; use antes
./tools/porta.sh --tudo    # de subir o stack, não com o jogo no ar)
./tools/ai_bench.py        # mede a IA sem ROS, centenas de partidas em segundos
./tools/duelo_bench.py     # regras do duelo + o tempo do Franky (--franky)
./tools/radio_test.py      # fala com a ponte sem ROS (linha de comando)
./tools/radio_console.py   # o mesmo, com teclado no navegador (:8060)
                           # + taxa de entrega, se a ponte tiver o tx_probe
./tools/painel.py          # TODO o diagnóstico numa tela só (:8062) — ele sobe
                           # o debug_panel e o flow_panel sozinho e derruba no
                           # Ctrl+C só o que ele subiu. Abra só o :8062.
```

**O `debug_panel` grava toda sessão em `~/.vss-game/logs/*.jsonl`** (ligado por
padrão, `--no-log` desliga; as 10 sessões mais recentes ficam, cada uma
rotacionando em partes de 64 MB). O arquivo **não** é o log da tela: a tela é
uma janela de 120 linhas que descarta o que acontece a 30 Hz (`OK |`, `TX id`,
`ID ALHEIO`) para não encher, e é justamente essa série temporal que responde
"o que mudou no instante em que parou". No arquivo vai a linha crua dos dois
lados, sem filtro, mais um retrato por segundo dos contadores e do veredito.
Medido: numa janela em que a tela guardou 2 linhas, o arquivo guardou 62.

```bash
ls -t ~/.vss-game/logs/ | head
jq -r 'select(.k=="raw") | "\(.h) \(.src) \(.line)"'                SESSAO.jsonl
jq -r 'select(.k=="snap") | "\(.h) rx_ok=\(.numbers.rx_ok)"'        SESSAO.jsonl
```

Ausência do registro `end` no fim do arquivo **é informação**: quer dizer que o
painel foi morto, não que ele parou sozinho.

Gravar firmware — use o `gravar.sh`, que descobre qual placa é qual sozinho:

```bash
./tools/gravar.sh feira           # o que joga: sem debug nos dois
./tools/gravar.sh debug           # o que o painel precisa: debug nos dois
./tools/gravar.sh probe           # ponte medindo entrega confirmada
./tools/gravar.sh debug --id 0    # grava o robô como robô 0 (o da IA)
```

**Não confie no `by-id` para escolher a placa.** O que o texto antigo aqui dizia
— "só a ponte tem número de série" — está errado, medido em 10/08/2026 com as
duas plugadas: as duas são CH340 (`1a86:7523`) **sem** serial, o udev gera o
mesmo nome e **só uma ganha o link**, sorteada a cada replug. Já foi vista
apontando para a ponte e para o robô. O jeito confiável é o **banner de boot**,
que é o que o `gravar.sh` e o `debug_panel.py` fazem — e ele só sai se a
abertura da serial resetar a placa (`stty ... hupcl`; o `arduino-cli monitor`
não reseta e por isso nunca mostra banner). Se precisar de um caminho estável,
`/dev/serial/by-path/` distingue por porta USB física.

Placa muda não é placa morta: parado, sem tráfego, o Nano não imprime nada.

No Pop!_OS, se o `/dev/ttyUSB*` não aparecer, é o `brltty` roubando o CH340 —
já foi resolvido com `systemctl mask brltty.service brltty-udev.service`.

**Sempre pare com `./tools/stop.sh`.** Matar por nome de processo deixa metade
do stack vivo, porque `cinematica`, `direction`, `joy_aggregator` e
`special_controls` são C++ e vivem em caminhos diferentes dos nós Python. Um
`simulator` órfão publicando `/game_data` junto com a câmera dá 74 Hz numa
câmera de 30 e posições alternando entre duas fontes — sintoma que não aponta
para a causa.

---

## Testando com os dois robôs de verdade

A ordem importa: cada passo só faz sentido se o anterior passou, e pular direto
para o jogo completo transforma cinco causas possíveis num sintoma só.

**1. Identidade.** Um robô por vez no USB (com a ponte), gravando do mesmo
fonte — o `--id` sobrescreve o `MY_ROBOT_ID` pelo compilador, então não há como
esquecer de trocar:

```bash
./tools/gravar.sh feira --id 0     # robô da IA, gol da esquerda
./tools/gravar.sh feira --id 1     # robô do visitante, gol da direita
```

O banner de boot que sai no fim confirma o id e o endereço (`VSS00`/`VSS01`).
Escreva o número no chassi: **dois robôs com o mesmo id andam juntos**, e o
sintoma parece problema de rádio.

**2. Link.** `./tools/gravar.sh debug` nos dois e `./tools/painel.py` (:8062).
Sem isto, um robô que não anda no passo 3 tem cinco explicações.

**3. As rodas, na mão, os dois ao mesmo tempo** — teclado num, gamepad no outro.
O teclado agora escolhe o robô (`keyboard_id`), então as duas fontes nunca
disputam o mesmo `/joy_N`:

```bash
ros2 launch startup teleop.py num_robots:=2 use_keyboard:=true \
    keyboard_id:=0 serial_port:=$(./tools/porta.sh)
```

Teclado → robô 0 (clique na janela "Teclado -> robô 0"; ESC pausa a captura),
gamepad `js0` → robô 1. `keyboard_id:=1` inverte. O gamepad é WASD equivalente:
volante no `axes[0]`, R2 anda, L2 dá ré.

**4. IA movendo robô, sem câmera.** O simulador publica `/game_data` e o rádio
manda o resultado para as rodas de verdade — o robô repete na bancada o que a
IA está fazendo na tela. Isola "a IA chega no motor" de "a visão enxerga":

```bash
ros2 launch startup game.py use_radio:=true serial_port:=$(./tools/porta.sh)
```

Suspenda as rodas: o robô físico não tem realimentação nenhuma da posição
simulada, então ele sai andando reto e o simulador não fica sabendo.

**5. O jogo inteiro.** Câmera publicando `/game_data`, IA no robô 0, visitante
no robô 1:

```bash
# visitante no gamepad
ros2 launch startup game.py use_vision:=true use_radio:=true use_joy:=true \
    serial_port:=$(./tools/porta.sh)

# ou no teclado, se não houver controle plugado
ros2 launch startup game.py use_vision:=true use_radio:=true use_keyboard:=true \
    serial_port:=$(./tools/porta.sh)
```

**A IA só liga durante a partida** (`/ai/enabled`, ver acima): comece o jogo
pelo operador em `:8090/operador` antes de concluir que a IA está morta.

Nunca ligue `use_joy` e `use_keyboard` juntos — os dois publicam no `/joy` do
visitante e o robô gagueja, sem nada no log explicando. Pela mesma razão o
teclado da GUI do simulador se cala sozinho quando um dos dois ocupa a vaga
dele.

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

**A cor tem medida, não só olhômetro.** A GUI mede a máscara de cada cor no
quadro de agora e mostra em barra. Existe porque faixa confortável e faixa
roçando são **idênticas na imagem** — nas duas o robô aparece detectado — e só
a segunda some quando alguém passa perto da luminária no meio da partida.

| medida | o que pega | onde dói |
|---|---|---|
| **folga** H/S/V | distância dos pixels à borda da faixa, no p5 | folga baixa = a etiqueta some com uma sombra |
| **solidez** | área do blob / área do fecho convexo | cai **antes** de a detecção falhar: a faixa apertada esburaca a máscara e o centróide pula |
| **limpeza** | pixels acesos fora dos blobs esperados | faixa larga acende feltro e cenário |
| **visto** | em que fração dos últimos 120 frames a cor apareceu | o "pisca", que o retrato instantâneo esconde |
| **colisão** | pixels em comum entre duas máscaras | duas cores brigando pelo mesmo pixel (foi assim que o azul-escuro morreu) |

Três coisas na tela que valem mais que os números: o botão **máscara** troca o
vídeo pela máscara da cor escolhida (verde = blob aceito, laranja = fora da
faixa de área — "não detecta nada" com um blob laranja de 140 px é `min_area`
alto demais, não cor errada); **passar o mouse** no vídeo diz o HSV do pixel e
por qual eixo cada cor o rejeitaria; e a **nuvem S×V** mostra de que lado a
nuvem encosta na moldura, que é o que decide qual slider mexer.

Depois de clicar numa cor sai o retrato da amostra — se a **cobertura** não for
~100%, o clique pegou a borda da etiqueta e a faixa abriu para caber o feltro
junto.

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
| `robot_communication` + `firmware/` | prontos; **link medido em 10/08: 300/300 pacotes a 30 Hz, 100%**, com endereço por robô e o robô 1 ignorando o que é do 0 |

Calibração salva em `~/.vss-game/vision.json` — proporção dos cantos fechando
em 0,10% do campo real.

**Falta para o jogo estar 100%** — ver `README.md` para o detalhe. A cadeia
do olho à roda já fechou pelo menos uma vez; o que resta é montagem e aferição:

1. ~~o `franky.ino` não filtra por `robot_id`~~ **RESOLVIDO em 10/08.** O
   `robot_rx` está gravado e validado: cada robô tem endereço próprio e o filtro
   por ID por cima. Testado nos dois sentidos — gravado como id 0 recebe só o
   que é do 0, como id 1 só o que é do 1. Falta apenas repetir com o **segundo
   robô físico**, com `./tools/gravar.sh feira --id 0` nele
2. `wheel_speed_max` (0,75) é chute declarado — medir cronometrando um metro
3. exatidão da visão nunca medida contra régua; o **ângulo** em especial nunca
   foi conferido contra referência física
4. `use_joy` nunca testado com gamepad plugado
