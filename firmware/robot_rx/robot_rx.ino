// robot_rx — firmware do robô VSS (Arduino Nano + nRF24L01 + TB6612FNG)
//
// Recebe comandos pelo rádio e aciona os motores. Cada robô só obedece aos
// pacotes endereçados ao seu próprio ID.
//
// ============================================================================
//  ANTES DE GRAVAR: ajuste MY_ROBOT_ID abaixo, um valor diferente por robô,
//  e escreva o número no chassi com fita/etiqueta. Dois robôs com o mesmo ID
//  vão se mexer juntos e o jogo não funciona.
// ============================================================================
//
// Wiring:
//   TB6612FNG: PWMA=D5, AIN1=D4, AIN2=D3, PWMB=D9, BIN1=D7, BIN2=D8, STBY=D2
//   nRF24L01:  CE=D6, CSN=D10

#include <RF24.h>
#include <SPI.h>
#include <stddef.h>
#include <stdint.h>

// ── Identidade deste robô ────────────────────────────────
// Também escolhe o endereço de rádio (ver ENDERECOS): precisa ser < MAX_ROBOS.
//
// O `#ifndef` é o que permite gravar os dois robôs do mesmo fonte, sem editar o
// arquivo entre uma gravação e outra — que é como se erra e acaba com dois
// robôs de mesmo ID andando juntos. O `tools/gravar.sh --id N` usa isto.
#ifndef MY_ROBOT_ID
#define MY_ROBOT_ID 1
#endif

// ── Debug ────────────────────────────────────────────────
// Serial.print custa tempo. Com DEBUG_RADIO ligado o loop fica mais lento e
// pode perder pacotes. Ligue para diagnosticar, desligue para jogar.
//
// O `#ifndef` permite ligar na linha de comando sem editar o arquivo, para que o
// default gravado na feira continue sendo o silencioso:
//   arduino-cli compile --build-property \
//       compiler.cpp.extra_flags=-DDEBUG_RADIO=1 firmware/robot_rx
#ifndef DEBUG_RADIO
#define DEBUG_RADIO 0
#endif
#ifndef DEBUG_RAW_PACKET
#define DEBUG_RAW_PACKET 0
#endif

// ── Radio ────────────────────────────────────────────────
RF24 radio(6, 10);

// Um endereço por robô, e não um endereço só para todos. Isto existe porque o
// auto-ACK é obrigatório neste link (ver o comentário em setAutoAck lá embaixo)
// e ACK só funciona com um receptor por endereço: dois robôs no mesmo endereço
// respondem juntos, as confirmações colidem no ar e a entrega despenca.
//
// Mesma tabela no `tx_bridge.ino` e no `tx_probe.ino`. Mudou aqui, mude nos três.
constexpr uint8_t MAX_ROBOS = 2;
const byte ENDERECOS[MAX_ROBOS][6] = {"VSS00", "VSS01"};

// Precisa bater com o TX e com o start_byte do nó ROS.
constexpr uint8_t START_BYTE = 0x14;

// Precisa bater byte a byte com shared_interfaces::RadioMessage (14 bytes).
#pragma pack(push, 1)
struct Message {
  uint8_t start_byte;
  float Motor1;
  float Motor2;
  int32_t robot_id;
  uint8_t checksum;
};
#pragma pack(pop)

constexpr size_t PACKET_SIZE = sizeof(Message);

static_assert(PACKET_SIZE == 14, "Message precisa ter 14 bytes");

// Sem isto, um MY_ROBOT_ID fora da tabela lê lixo da memória como endereço e o
// robô escuta um endereço aleatório — que é indistinguível de rádio quebrado.
static_assert(MY_ROBOT_ID >= 0 && MY_ROBOT_ID < MAX_ROBOS,
              "MY_ROBOT_ID precisa estar dentro de ENDERECOS (0..MAX_ROBOS-1)");

// ── Motor pins ───────────────────────────────────────────
#define PWMA 5
#define AIN1 4
#define AIN2 3
#define PWMB 9
#define BIN1 7
#define BIN2 8
#define STBY 2

// Abaixo deste PWM o motor só chia e esquenta sem girar. Zera para não
// desperdiçar corrente nem fazer barulho parado.
constexpr int PWM_DEADZONE = 25;

// ── Compensação: roda A que não gira para frente ─────────
// Paliativo para um robô cujo canal A do driver perdeu o sentido "frente" —
// AIN1 sem contato, meia-ponte queimada, ou o motor travado num sentido. O
// sintoma que diagnostica isto é bem específico: as duas teclas em que a roda A
// vai para frente (andar reto para frente, girar para a direita) saem curvando,
// e as duas em que ela vai para trás (ré, girar para a esquerda) saem perfeitas.
//
// Ligue com `./tools/gravar.sh feira --id 1 --roda-a-sem-frente`. Deixe
// desligado no robô são: a compensação custa a ré, e num robô inteiro isso é
// perda pura.
//
// ISTO NÃO CONSERTA O ROBÔ. Falta um grau de liberdade e nenhum software o
// devolve; o que dá para fazer é escolher quais movimentos sobrevivem. Ver
// `compensaRodaA` para o que se ganha e o que se perde.
#ifndef RODA_A_SEM_FRENTE
#define RODA_A_SEM_FRENTE 0
#endif

// ── Compensação: roda A com os fios trocados ─────────────
// Para um robô que saiu da montagem com os dois fios do motor A invertidos no
// TB6612 (AO1/AO2). O sintoma não parece fiação: andar para frente e dar ré
// viram GIRO para lados opostos, porque as duas rodas sempre se opõem. Girar,
// esse sim, anda reto.
//
// O `robot_selftest` NÃO pega isto com o robô suspenso: os motores são montados
// espelhados nas duas laterais, então "as duas rodas girando" parece certo no
// ar, e o passo "as duas para frente (robô anda reto)" é justamente o que não
// se julga sem o robô no chão.
//
// Medido em 13/08/2026 no robô 1: com o rádio entregando `PWM_A: 114` positivo
// (conferido no log do próprio robô, DEBUG_RADIO ligado), a roda A empurrava
// para trás; a roda B obedecia certo.
//
// Ao contrário de RODA_A_SEM_FRENTE, esta compensação NÃO CUSTA NADA: inverter
// o sinal é exato e os quatro movimentos ficam inteiros. Ainda assim o conserto
// de verdade é trocar os dois fios no chassi e gravar sem a flag — enquanto ela
// existir, este é o robô que não anda com o firmware padrão, e isso é uma
// pegadinha esperando a próxima pessoa que gravar.
//
// Ligue com `./tools/gravar.sh feira --id 1 --inverte-roda-a`.
#ifndef INVERTE_RODA_A
#define INVERTE_RODA_A 0
#endif

// ── Command timeout ──────────────────────────────────────
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 1000;

// ── Checksum ─────────────────────────────────────────────
// XOR de Motor1 até robot_id. Ignora start_byte e o próprio checksum.
uint8_t calculateChecksum(const Message &pkt) {
  uint8_t checksum = 0;

  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&pkt);

  const size_t startOffset = offsetof(Message, Motor1);
  const size_t checksumOffset = offsetof(Message, checksum);

  for (size_t i = startOffset; i < checksumOffset; i++) {
    checksum ^= bytes[i];
  }

  return checksum;
}

void printPacketHex(const uint8_t *data, size_t size) {
#if DEBUG_RAW_PACKET
  Serial.print("RAW: ");

  for (size_t i = 0; i < size; i++) {
    if (data[i] < 16) {
      Serial.print("0");
    }

    Serial.print(data[i], HEX);
    Serial.print(" ");
  }

  Serial.println();
#else
  (void)data;
  (void)size;
#endif
}

// ── Motor functions ──────────────────────────────────────
void motorA(int speed) {
  if (abs(speed) < PWM_DEADZONE) speed = 0;

  if (speed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }

  analogWrite(PWMA, constrain(speed, 0, 255));
}

void motorB(int speed) {
  if (abs(speed) < PWM_DEADZONE) speed = 0;

  if (speed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }

  analogWrite(PWMB, constrain(speed, 0, 255));
}

void stopAll() {
  motorA(0);
  motorB(0);
}

void enableMotors() { digitalWrite(STBY, HIGH); }

void disableMotors() { digitalWrite(STBY, LOW); }

// ── Compensação da roda A ────────────────────────────────
// Duas transformações, nesta ordem.
//
// 1. TROCA A FRENTE DO ROBÔ. Se a roda A só anda para trás, o único movimento
//    reto que sobra é com as duas rodas para trás — que é justamente a tecla que
//    ainda funciona hoje. Então passamos a chamar a traseira física de frente:
//    o robô gira 180°, a roda A vira a da direita, e o sentido inverte. Daí
//    `(Motor1, Motor2) = (-Motor2, -Motor1)`. Andar para frente volta a andar.
//
// 2. SATURA O QUE A RODA NÃO ENTREGA. Depois da troca, o impossível é Motor1
//    positivo. Pedidos assim são projetados no vizinho viável:
//
//      girar para a direita  ->  roda A parada, roda B para trás. Deixa de ser
//                                giro no próprio eixo e vira pivô sobre a roda
//                                A — gira mais devagar e ocupa mais espaço, mas
//                                gira para o lado certo.
//      ré                    ->  parado. Não existe aproximação honesta de
//                                "andar reto para lá" com uma roda só; virar
//                                isso num pivô enganaria quem está no controle,
//                                que é pior que a tecla não fazer nada.
//
// Sobrevivem intactos: andar para frente, girar para a esquerda. Perde-se a ré.
//
// ATENÇÃO À CÂMERA. Trocar a frente aqui desalinha o firmware da visão: ela lê
// a orientação pelo vetor retângulo->quadrado das etiquetas, e o robô passaria a
// andar 180° fora do que a câmera reporta — a IA e o `game_master` iriam junto.
// Se este robô for entrar em jogo com visão, GIRE AS ETIQUETAS 180° NO CHASSI
// junto com esta flag. Para bancada e painel, sem câmera no circuito, não muda
// nada.
void compensaRodaA(float &m1, float &m2) {
  const float trocado1 = -m2;
  const float trocado2 = -m1;

  if (trocado1 > 0.0f && trocado2 > 0.0f) {
    m1 = 0.0f;
    m2 = 0.0f;
    return;
  }

  m1 = trocado1 > 0.0f ? 0.0f : trocado1;
  m2 = trocado2;
}

// ── Handle received packet ───────────────────────────────
void handlePacket(const Message &pkt) {
  // Motor1 e Motor2 chegam em [-1.0, 1.0]. O nó ROS já faz o clamp, mas se um
  // pacote corrompido passar pelo checksum o constrain evita PWM absurdo.
  float m1 = pkt.Motor1;
  float m2 = pkt.Motor2;

  // Vem antes de qualquer outra compensação: as demais raciocinam sobre o
  // sentido REAL da roda ("a roda A não vai para frente"), então precisam do
  // sinal já normalizado para não decidir pelo avesso.
#if INVERTE_RODA_A
  m1 = -m1;
#endif

#if RODA_A_SEM_FRENTE
  compensaRodaA(m1, m2);
#endif

  int speedA = static_cast<int>(m1 * 255.0f);
  int speedB = static_cast<int>(m2 * 255.0f);

  speedA = constrain(speedA, -255, 255);
  speedB = constrain(speedB, -255, 255);

  motorA(speedA);
  motorB(speedB);

  lastCommandTime = millis();

#if DEBUG_RADIO
  Serial.print("OK | M1: ");
  Serial.print(pkt.Motor1, 3);

  Serial.print(" | M2: ");
  Serial.print(pkt.Motor2, 3);

  Serial.print(" | PWM_A: ");
  Serial.print(speedA);

  Serial.print(" | PWM_B: ");
  Serial.println(speedB);
#endif
}

// ── Setup ────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  enableMotors();
  stopAll();

  radio.begin();

  radio.openReadingPipe(1, ENDERECOS[MY_ROBOT_ID]);

  // Canal e data rate precisam bater com o TX. Deixamos explícito nos dois lados
  // para não depender do default da biblioteca.
  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);

  // ============================================================================
  //  NÃO DESLIGUE O AUTO-ACK. Medido em 10/08/2026, com o robô a 1 m da ponte:
  //
  //      TX ACK off + RX ACK off  ->    0,0%      (era o firmware anterior)
  //      TX ACK off + RX ACK on   ->    0,0%
  //      TX ACK on  + RX ACK off  ->    0,0%
  //      TX ACK on  + RX ACK on   ->  100,0%
  //
  //  Zero, não "ruim": com auto-ACK desligado em QUALQUER um dos lados este
  //  link não entrega um único pacote, e sem erro nenhum nos dois logs. O robô
  //  fica parado e o sintoma não aponta para cá. Foi o que segurou o projeto.
  //
  //  O preço é que ACK exige um receptor por endereço — daí a tabela ENDERECOS.
  // ============================================================================
  radio.setAutoAck(true);

  radio.startListening();

  Serial.print("robot_rx | ID: ");
  Serial.print(MY_ROBOT_ID);
  Serial.print(" | endereço: ");
  Serial.println((const char *)ENDERECOS[MY_ROBOT_ID]);
  Serial.println(radio.isChipConnected() ? "radio OK" : "radio FAIL");

#if RODA_A_SEM_FRENTE
  // No banner de propósito: sem isto, um robô gravado com a compensação vira,
  // meses depois, um "robô que não dá ré" sem nada no código que explique.
  // O `gravar.sh` lê os banners no fim, então o aviso aparece na hora de gravar.
  Serial.println("COMPENSACAO: roda A sem frente (frente trocada, SEM re)");
#endif

#if INVERTE_RODA_A
  // Pelo mesmo motivo do aviso acima, e aqui ainda mais: esta compensação não
  // tira nada, então um robô gravado com ela se comporta perfeitamente — e é
  // exatamente por isso que ninguém desconfia dela quando o robô, regravado sem
  // a flag, volta a girar no lugar do nada.
  Serial.println("COMPENSACAO: roda A invertida (fios trocados no chassi)");
#endif

#if DEBUG_RADIO
  Serial.print("sizeof(Message): ");
  Serial.println(sizeof(Message));

  Serial.print("START_BYTE: 0x");
  Serial.println(START_BYTE, HEX);
#endif
}

// ── Loop ─────────────────────────────────────────────────
void loop() {
  // Para os motores se ficar muito tempo sem comando válido. Vale tanto para
  // queda de rádio quanto para o notebook travar no meio de uma partida.
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    stopAll();
  }

  if (!radio.available()) {
    return;
  }

  Message packet;
  radio.read(&packet, sizeof(packet));

  printPacketHex(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));

  if (packet.start_byte != START_BYTE) {
#if DEBUG_RADIO
    Serial.print("START BYTE INVALIDO: 0x");
    Serial.println(packet.start_byte, HEX);
#endif
    return;
  }

  uint8_t expected = calculateChecksum(packet);

  if (packet.checksum != expected) {
#if DEBUG_RADIO
    Serial.print("CHECKSUM FAIL | recebido: 0x");
    Serial.print(packet.checksum, HEX);
    Serial.print(" esperado: 0x");
    Serial.println(expected, HEX);
#endif
    return;
  }

  // Todos os robôs escutam o mesmo endereço e canal, então cada um filtra por ID.
  // Sem isto, os quatro robôs obedecem ao mesmo comando ao mesmo tempo.
  if (packet.robot_id != MY_ROBOT_ID) {
#if DEBUG_RADIO
    // Sem este aviso o descarte por ID é indistinguível de "não chegou nada" —
    // e os dois têm o mesmo sintoma: robô parado, log vazio.
    Serial.print("ID ALHEIO: ");
    Serial.println(packet.robot_id);
#endif
    return;
  }

  handlePacket(packet);
}
