// tx_probe — ponte serial -> rádio COM confirmação de entrega.
//
// Mesmo protocolo de 14 bytes do tx_bridge, mas com uma diferença que é todo o
// ponto deste sketch: aqui o auto-ACK fica **ligado**, e o resultado de cada
// `radio.write()` volta para o PC pela serial.
//
// Serve para responder "o robô recebeu?" sem depender de ver o robô andar —
// que é a única pergunta que importa quando o robô não se mexe e você não sabe
// se o problema é rádio, firmware, motor ou bateria.
//
// COMO FUNCIONA: o nRF24 implementa Enhanced ShockBurst. Com auto-ACK ligado, o
// receptor confirma cada pacote em hardware, sem uma linha de código do lado
// dele. O `radio.write()` devolve `true` só se essa confirmação chegou. Ou
// seja: `true` prova que o outro rádio recebeu e respondeu.
//
// NÃO É O SKETCH DA FEIRA. Use o `tx_bridge.ino` em jogo:
//   - auto-ACK só funciona com UM receptor; com vários robôs no mesmo endereço
//     as confirmações colidem entre si e a taxa despenca
//   - cada write() sem ACK bloqueia o loop pelas retentativas
// Aqui isso é aceitável porque o objetivo é medir, com um robô na bancada.
//
// Wiring: nRF24L01 CE=D6, CSN=D10

#include <RF24.h>
#include <SPI.h>
#include <stddef.h>
#include <stdint.h>

RF24 radio(6, 10);

// Precisa ser a mesma tabela do `tx_bridge.ino` e do `robot_rx.ino`, senão o
// probe mede um link que não é o do jogo — e mede 0%, sem que nada esteja errado.
constexpr uint8_t MAX_ROBOS = 2;
const byte ENDERECOS[MAX_ROBOS][6] = {"VSS00", "VSS01"};

int32_t destino_atual = -1;

constexpr uint8_t START_BYTE = 0x14;

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

uint8_t buffer[PACKET_SIZE];
size_t bufferIndex = 0;

uint16_t sent = 0;
uint16_t acked = 0;
unsigned long lastReport = 0;
const unsigned long REPORT_MS = 500;

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

void setup() {
  Serial.begin(115200);

  bool began = radio.begin();

  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);

  // Ligado de propósito — é o que gera a confirmação.
  radio.setAutoAck(true);

  // 5 retentativas a 1,25 ms. O default da biblioteca é 15, o que trava o loop
  // por ~60 ms sempre que o robô está fora do ar — e a 30 Hz isso é o pacote
  // inteiro perdido. 5 dá margem de sobra num link bom e mantém a ponte viva.
  radio.setRetries(5, 5);

  radio.stopListening();

  // Distingue os dois "não funciona" que parecem iguais de fora:
  //   RADIO FAIL  -> o chip nem responde ao SPI aqui (fio, alimentação)
  //   RADIO OK + entrega 0% -> este rádio está vivo, o do robô é que não
  //                            responde (robô desligado, canal, endereço)
  Serial.println(began && radio.isChipConnected() ? "PROBE RADIO OK"
                                                  : "PROBE RADIO FAIL");
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t byteRead = Serial.read();

    if (bufferIndex == 0 && byteRead != START_BYTE) {
      continue;
    }

    buffer[bufferIndex++] = byteRead;

    if (bufferIndex < PACKET_SIZE) {
      continue;
    }

    Message *pkt = reinterpret_cast<Message *>(buffer);

    if (pkt->checksum == calculateChecksum(*pkt)) {
      if (pkt->robot_id < 0 || pkt->robot_id >= MAX_ROBOS) {
        Serial.print("ID FORA DA TABELA: ");
        Serial.println(pkt->robot_id);
        bufferIndex = 0;
        continue;
      }

      if (pkt->robot_id != destino_atual) {
        radio.openWritingPipe(ENDERECOS[pkt->robot_id]);
        destino_atual = pkt->robot_id;
      }

      bool ok = radio.write(buffer, PACKET_SIZE);
      sent++;
      if (ok) {
        acked++;
      }
    } else {
      // Ressincroniza a partir do próximo start byte, como no tx_bridge: um
      // byte perdido não pode derrubar vários pacotes seguidos.
      size_t resync = 0;
      for (size_t i = 1; i < PACKET_SIZE; i++) {
        if (buffer[i] == START_BYTE) {
          resync = i;
          break;
        }
      }
      if (resync > 0) {
        bufferIndex = PACKET_SIZE - resync;
        memmove(buffer, buffer + resync, bufferIndex);
        continue;
      }
    }

    bufferIndex = 0;
  }

  // Relatório agregado. Uma linha por pacote a 30 Hz encheria a serial e
  // roubaria tempo do próprio envio.
  unsigned long now = millis();
  if (now - lastReport >= REPORT_MS) {
    lastReport = now;
    Serial.print("ACK ");
    Serial.print(acked);
    Serial.print('/');
    Serial.println(sent);
    sent = 0;
    acked = 0;
  }
}
