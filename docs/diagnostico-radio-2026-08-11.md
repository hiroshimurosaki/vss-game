# Por que o rádio do robô morre — diagnóstico de 11/08/2026

Investigação da falha "o robô para de responder e não volta". Este documento
registra o que foi medido, o que foi descartado, e por quê — inclusive as duas
hipóteses erradas que os dados derrubaram, porque saber por que elas caíram é o
que impede de voltar a elas.

---

## Resumo

| | |
|---|---|
| **Suspeita inicial** | Ruído/queda de tensão do motor corrompe os pacotes; erros de checksum aparecem quando a bateria é ligada e os motores ficam mais fortes. |
| **O que os dados mostraram** | Corrente de motor **não** mata o rádio. Estol a 100% — o pior caso elétrico possível — passou com 98,7% de entrega. Quem mata é **movimento mecânico do robô**, com corrente zero. |
| **Diagnóstico** | Contato marginal (intermitente) no caminho de alimentação do módulo nRF24, sensível a inércia, **não** às pontas dupont acessíveis à mão. |
| **Estado** | Localização exata do contato ainda **não** determinada. Bloco de ensaios de percussão escrito para isso, ainda não rodado. |

A falha é **permanente**: uma vez disparada, o rádio não volta sozinho — na
primeira sessão ficou morto por 485 s até intervenção manual. Isso tem
consequência de método e está tratado na seção 5.

---

## 1. O sintoma

Robô para de responder no meio da operação. Nos logs do firmware aparecem
`CHECKSUM FAIL` em rajada. O operador observou que os erros só apareciam com a
**bateria ligada**, condição em que os motores ficam visivelmente mais fortes —
daí a hipótese inicial de ruído elétrico dos motores.

O robô fica permanentemente ligado ao PC por USB durante a bancada, para que se
possa ler o terminal.

## 2. Por que a tela do painel não bastava

O `debug_panel` mostrava uma janela de 120 linhas em memória, que:

- **some junto com o processo** — nada sobrevive para autópsia;
- **descarta de propósito o que acontece a 30 Hz** (`OK |`, `TX id`,
  `ID ALHEIO`), senão a tela enche em segundos.

O problema é que é exatamente essa série temporal descartada que responde "o que
mudou no instante em que parou". Medido: numa mesma janela de observação, a tela
guardou **2 linhas**; o arquivo guardou **62**.

Foi implementada gravação em disco (`~/.vss-game/logs/*.jsonl`, ligada por
padrão), com quatro tipos de registro:

| `k` | quando | conteúdo |
|---|---|---|
| `meta` | abertura, e no topo de cada parte rotacionada | portas, banners, `argv` |
| `raw` | **toda** linha das duas seriais, sem filtro | `{t, h, src, line}` |
| `snap` | 1 Hz | contadores + veredito do painel |
| `end` | encerramento limpo | ausência dele **é informação**: o painel foi morto |

O tap de gravação fica no `LineReader`, **antes** do parser, porque os handlers
retornam cedo nos casos de 30 Hz — gravar depois deles herdaria os furos que
existem para poupar a tela.

## 3. Anatomia da falha

Sessão de 14:49:49, primeiros 421 s. O robô estava no USB e a ponte transmitindo
a 30 Hz o tempo todo.

| janela | TX ponte | `OK` no robô | erro | ACK do rádio |
|---|---|---|---|---|
| 0–82 s | 30/s | 30/s | zero | **100%** |
| +82 s | 30/s | 2 | 211 START BYTE | some 3 s — **reboot** |
| 85–112 s | 30/s | 30/s | zero | 100% |
| +112–121 s | 30/s | cai | — | **reboots; 6 em 0,6 s** |
| 122–135 s | 30/s | **0** | **30 CHECKSUM/s (100%)** | agonizando |
| **135 s →** | 30/s | **0** | **450 START BYTE/s** | **zero** |

Totais da sessão inteira (1918 s): 805.601 `START BYTE`, 3.440 `OK`, 357
`CHECKSUM FAIL`, 7 banners de boot do robô, e **3.810 ACK contra 53.224 SEM
ACK**.

A degradação é **monotônica**: saudável → reboots → checksum → zeros → nunca
volta. Nenhuma oscilação, nenhuma recuperação espontânea.

## 4. Quatro evidências contra ruído de motor

**1. O nRF24 tem CRC próprio, em hardware.** Pacote corrompido no ar é descartado
pelo chip e nunca chega à verificação de checksum da aplicação. Um `CHECKSUM
FAIL` no firmware significa corrupção **depois** do rádio — no barramento SPI ou
dentro do Nano. Isso já aponta para dentro do robô, não para o ar.

**2. Os bytes são constantes degeneradas, não lixo.** Ruído produz variedade:

```
CHECKSUM FAIL | recebido: 0x1 esperado: 0x16   ← recebido SEMPRE 0x1
CHECKSUM FAIL | recebido: 0x1 esperado: 0x8D      (357 ocorrências, sem exceção)
START BYTE INVALIDO: 0x0                       ← SEMPRE 0x0
                                                  (805.601 ocorrências, valor único)
```

**3. 450 START BYTE/s com 30 pacotes/s chegando — 15× a mais.** O `loop()` do
`robot_rx` só imprime se `radio.available()` for verdadeiro. Leitura SPI
retornando tudo zero faz o registrador `FIFO_STATUS` ler `0x00`, e o bit
`RX_EMPTY` em zero significa "tem dado" — **`available()` trava em verdadeiro
para sempre**, lendo zeros. Não é pacote chegando errado: o Nano parou de
conversar com o nRF24.

**4. A ponte confirma pelo outro lado.** O auto-ACK é respondido pelo **hardware**
do nRF24, sem passar pelo firmware. Rádio que não ACKa não é bug de código — é
chip sem alimentação ou sem comunicação.

Some-se a isso que, no instante da tempestade de reboots, o comando de motor era
`M1 = M2 = +0,15` — PWM 38 de 255. Ruído de motor acompanharia o comando; não
acompanha.

## 5. O roteiro de ensaios

Duas hipóteses sobreviveram à seção 4 e produzem **exatamente o mesmo gráfico**:

- **(a)** queda de tensão pelo surto de corrente dos motores;
- **(b)** conexão intermitente no caminho de alimentação.

Elas se confundem porque *ligar a bateria* é também *mexer no robô*. Separar as
duas exige ensaios que apliquem **carga sem tocar** no robô e ensaios que
**toquem sem aplicar carga**.

Três restrições de método vieram dos dados:

1. **A falha trava.** Um ensaio rodado com o robô já morto não mede nada, e um
   roteiro sem conferência de saúde entre passos produz doze "falhou" sem
   significado. Daí o portão de saúde antes e depois de cada ensaio.
2. **Ordem importa.** Do mais brando ao mais agressivo; o ensaio de estol fica
   por último porque é o mais provável de matar, e o que morre no fim não
   contamina o resto.
3. **Carga aplicada por software.** Acelerador e duração exatos valem mais que a
   mão humana quando o objetivo é comparar um ensaio com outro.

A ferramenta (`tools/ensaio.py`) conduz o roteiro, aplica a carga pelo HTTP do
painel, e carimba início/fim no **mesmo relógio** do log do painel. O
`--analisar` cruza os dois.

## 6. Resultados

```
ensaio              carga    ACK%   perfil       veredito
ref-usb-parado          —   100.0   ██████       limpo
ref-usb-motor         1.0   100.0   ██████       limpo
bat-liga                —    61.0   ▒█████▒░░    degradou ao ligar, voltou, morreu depois
bat-repouso             —   100.0   ██           limpo
mexer-fios-radio        —    99.9   ██████       limpo
mexer-fios-alim         —   100.0   ██████       limpo
mexer-robo              —    28.9   █▓░░░░       *** MATOU O ROBÔ ***
ar-25 / ar-50    0.25/0.5   100.0   ██████       limpo
ar-100                1.0    99.0   █▓████       sobreviveu
chao-100              1.0    97.9   ▓█▓███       sobreviveu
reversao            ±100%    86.1   ▓▓██         sobreviveu
travado               1.0    98.7   ██           sobreviveu
```

`█` ≥95% de ACK · `▓` >50% · `▒` >5% · `░` ≤5%, em blocos de 5 s.

Três leituras:

**Corrente de motor não mata.** Estol a 100% com as rodas travadas — o pior caso
elétrico que existe neste robô — passou com 98,7%. Reversão brusca, 86%. Chão a
100%, 97,9%. A hipótese (a) cai.

**Movimento mata.** Balançar o robô de leve, motor parado, corrente zero:
100% → 0%, e ficou morto.

**O perfil do `bat-liga` fecha o caso.** `▒█████▒░░`: caiu **no instante** em que
a chave foi acionada, recuperou sozinho, rodou limpo por ~25 s, e só então
morreu. Surto de corrente de ligação mataria em `t = 0` e não recuperaria. Isso é
assinatura de contato intermitente.

**E o contato não está nas pontas dupont.** Balançar os fios do rádio (99,9%) e
os fios de alimentação (100%) foi limpo; balançar o robô inteiro matou. O que
distingue os dois é **inércia**: balançar fios move os fios; balançar o robô
acelera a *massa* do módulo no header, do Nano no soquete, da bateria no suporte.

## 7. Conclusão

Contato marginal no caminho de alimentação do nRF24, sensível a aceleração,
localizado em algum ponto que **não** é acessível pelo teste de balançar fios.
Uma interrupção breve leva o chip a um estado travado do qual ele não sai
sozinho, e do qual o `setup()` do firmware também não o tira.

Consequência prática: **os capacitores deixaram de ser a primeira ação.**
Continuam valendo (ver seção 8), mas tratam o sintoma.

## 8. Onde capacitor agrega

Um capacitor é um **reservatório local de energia**, e obedece a três regras que
decidem onde ele vai:

1. **Só segura o nó em que está, e só a jusante da interrupção.** Se o contato
   ruim está entre a bateria e o rádio, um capacitor *antes* do contato não
   entrega nada ao rádio — a corrente teria que atravessar justamente o ponto que
   abriu. Por isso um capacitor "na entrada de energia" é quase inútil para esta
   falha específica.
2. **Serve para transiente, não para queda contínua.** Contato com resistência
   alta mas constante produz queda DC, e contra isso o capacitor não faz nada:
   ele é um pulmão, não uma fonte.
3. **Quanto mais perto do consumidor, melhor.** Perna longa é indutor, e anula o
   capacitor exatamente na frequência do pico que se quer matar.

### Quanto tempo ele segura

`t = C · ΔV / I`. O nRF24 puxa ~15 mA ouvindo e opera de 3,3 V até ~1,9 V:

| capacitor no VCC do rádio | queda tolerada | segura por |
|---|---|---|
| 10 µF | 0,5 V | 0,3 ms |
| 100 µF | 0,5 V | 3,3 ms |
| 100 µF | 1,4 V (limite) | 9,3 ms |
| 470 µF | 1,4 V | 44 ms |

Para o Nano (~20 mA, reset em ~2,7 V): 100 µF no 5 V seguram ~2,5 ms; 470 µF,
~12 ms.

**Capacitor compra milissegundos.** Cobre repique de contato, transiente de
chaveamento e microinterrupção. Não cobre um fio aberto por 50 ms.

Ainda assim pode bastar aqui, por um motivo específico: a falha **trava**. Um
glitch de microssegundos produziu um travamento de centenas de segundos. Se o
capacitor impedir o glitch, impede o travamento — o remédio é desproporcional à
doença, a favor.

### Onde colocar

```
                        ┌── ponto-estrela de GND (um só) ──┐
                        │                                  │
bateria ──[chave]──┬────┴── VM do TB6612 ──► motores       │
                   │        ‖ 470–1000 µF  ‖ 100 nF        │
                   │                                        │
                   └── VIN do Nano                          │
                       ‖ 100–470 µF  ‖ 100 nF               │
                                                            │
   3V3 ────────────────► VCC do módulo nRF24 ───────────────┘
                        ‖ 100 µF  ‖ 100 nF   ← o que importa nesta falha
                          soldados NOS PINOS do módulo
```

Mais 100 nF cerâmico direto nos terminais de cada motor — mata ruído de escova na
origem, que é a única parte em que "ruído" tem papel real neste robô.

### Montagem

- **Sempre os dois em paralelo.** O eletrolítico dá volume mas é lento; o
  cerâmico de 100 nF é rápido mas pequeno. Um não substitui o outro.
- **Pernas curtas**, no limite do que der para soldar. É o erro que mais anula o
  esforço.
- **Polaridade**: a listra do eletrolítico é o negativo, vai no GND.
- **Tensão nominal com folga de 2×**: 16 V num trilho de 3,3 V ou 5 V.
- **GND num ponto-estrela único.** Terra em cadeia entre motor, Nano e rádio faz
  a corrente do motor passar pelo terra do rádio.
- Atenção à unidade: **100 mF são 100.000 µF**, um banco de supercapacitor. O
  valor pretendido é 100 µF.

## 9. O que fazer

Em ordem de retorno:

1. **Rodar `./tools/ensaio.py --percussao`** para localizar a junta. Um ponto por
   ensaio, 20 s cada, motor parado. O ensaio `perc-bancada` — bater na bancada
   *sem tocar no robô* — é o mais limpo: prova contato marginal sem nenhuma
   variável de mão ou de cabo.
2. **Ressoldar/reassentar** o ponto que o passo 1 apontar.
3. **Capacitores** conforme a seção 8, começando pelo do módulo.
4. **Watchdog de rádio no `robot_rx.ino`**: detectar `available()` verdadeiro com
   start byte inválido N vezes seguidas, ou ausência de pacote válido por X ms, e
   refazer `radio.begin()` + reconfiguração. Cobre a faixa de tempo que o
   capacitor não alcança e transforma centenas de segundos de robô morto em ~1 s
   de engasgo — independentemente de qual seja a causa raiz.

O passo 4 é o mais importante para a feira. Com `DEBUG_RADIO` desligado (que é
como o robô joga), esta falha se manifesta em **silêncio absoluto**: sem serial,
sem erro, robô parado e nada em log nenhum.

## 10. O que ainda não se sabe

- **A localização exata do contato.** É o que o bloco de percussão resolve.
- **Confundimento residual**: todos os ensaios que mataram foram com o robô
  **preso ao USB**; os que sobreviveram sob carga foram com ele **solto**.
  Balançar os fios (também preso) ficou limpo, o que enfraquece bastante o cabo
  como explicação, mas não o elimina. O `perc-bancada` também resolve isso,
  porque não puxa cabo nenhum.
- **Amostra**: um robô, uma bancada, uma sessão de ensaios. O segundo robô físico
  nunca foi submetido a nada disso.
- Nenhuma medida de tensão com instrumento foi feita. Todo o diagnóstico elétrico
  é inferido de comportamento digital.

---

## Apêndice A — reproduzir

```bash
./tools/painel.py                  # terminal 1: sobe o painel e grava
./tools/ensaio.py                  # terminal 2: conduz o roteiro e carimba
./tools/ensaio.py --percussao      # bloco 7: localiza a junta
./tools/ensaio.py --recuperacao    # bloco 6: só depois que morreu
./tools/ensaio.py --analisar       # cruza carimbos com log e imprime a tabela
```

Feche a aba do navegador antes de rodar o `ensaio.py`: dois clientes mandando
comando alternado viram "pacote fantasma" no log.

Leitura direta do log:

```bash
ls -t ~/.vss-game/logs/ | head
jq -r 'select(.k=="raw")  | "\(.h) \(.src) \(.line)"'          SESSAO.jsonl
jq -r 'select(.k=="snap") | "\(.h) rx_ok=\(.numbers.rx_ok)"'   SESSAO.jsonl
```

## Apêndice B — dois defeitos de ferramenta encontrados no caminho

Registrados porque ambos produziram **conclusões erradas com cara de dado**.

**1. O painel somava ACK e SEM ACK no mesmo contador.** A linha `TX id 1 SEM ACK`
termina em `ACK`; qualquer teste ingênuo de sufixo conta a falha como sucesso.
Corrigido testando o negativo primeiro, e o painel passou a expor `ack` e `noack`
separados. Esse era o único sinal capaz de sustentar o diagnóstico final.

**2. O `ensaio.py` julgava vida pela serial do robô.** Nos ensaios em que o robô
anda solto ele sai do USB, o painel fica sem linhas dele, e "sem serial" foi lido
como "morto" — seis ensaios foram marcados INVÁLIDO quando os seis haviam
passado com 86–100% de ACK. O sinal primário passou a ser o ACK, que vem da
ponte e sobrevive ao robô sair do cabo.

A lição comum às duas: **o sinal de saúde tem que sobreviver à condição de
teste.** Um indicador que só funciona na bancada não serve para julgar o campo.

## Apêndice C — numeração de `/dev/ttyUSB*` durante a investigação

Um `debug_panel` órfão de sessão anterior manteve aberto o fd de um
`/dev/ttyUSB0` já removido. Enquanto o fd existe, o kernel não libera o minor, e
toda placa plugada depois cai em `ttyUSB1` — quebrando o default de várias
ferramentas. Diagnóstico: `ls -l /proc/*/fd | grep ttyUSB` mostra o culpado com o
sufixo `(deleted)`.

Isso somou-se à armadilha já conhecida (as duas placas são CH340 sem número de
série, e o `by-id` sorteia qual ganha o link). Ao longo desta investigação as
portas circularam por `ttyUSB0/1`, `ttyUSB2/3` e `ttyUSB3/0`. **O único critério
confiável de identidade continua sendo o banner de boot**, que é o que o
`gravar.sh` e o `debug_panel` usam.
