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
#define MY_ROBOT_ID 1

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
const byte address[6] = "00001";

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

// ── Handle received packet ───────────────────────────────
void handlePacket(const Message &pkt) {
  // Motor1 e Motor2 chegam em [-1.0, 1.0]. O nó ROS já faz o clamp, mas se um
  // pacote corrompido passar pelo checksum o constrain evita PWM absurdo.
  int speedA = static_cast<int>(pkt.Motor1 * 255.0f);
  int speedB = static_cast<int>(pkt.Motor2 * 255.0f);

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

  radio.openReadingPipe(1, address);

  // Canal e data rate precisam bater com o TX. Deixamos explícito nos dois lados
  // para não depender do default da biblioteca.
  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);

  // Precisa espelhar o tx_bridge, que desliga o auto-ACK. O TX é broadcast para
  // vários robôs no mesmo endereço e ninguém confirma; se o RX continuar com
  // auto-ACK ligado, ele gasta tempo de ar respondendo a cada pacote para um
  // transmissor que não está ouvindo — e com mais de um robô em campo essas
  // respostas colidem entre si.
  radio.setAutoAck(false);

  radio.startListening();

  Serial.print("robot_rx | ID: ");
  Serial.println(MY_ROBOT_ID);
  Serial.println(radio.isChipConnected() ? "radio OK" : "radio FAIL");

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
