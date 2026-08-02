// tx_bridge — ponte serial -> rádio (Arduino Nano + nRF24L01)
//
// Fica ligado no notebook por USB. Lê pacotes de 14 bytes que o nó ROS
// `radio_communication` escreve na serial e retransmite por rádio para os robôs.
// Não interpreta o conteúdo: só valida e repassa.
//
// Wiring: nRF24L01 CE=D6, CSN=D10

#include <RF24.h>
#include <SPI.h>
#include <stddef.h>
#include <stdint.h>

#define DEBUG_TX 0

RF24 radio(6, 10);
const byte address[6] = "00001";

constexpr uint8_t START_BYTE = 0x14;

// Precisa bater byte a byte com shared_interfaces::RadioMessage e com o robot_rx.
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

  radio.begin();

  // Mesmos parâmetros do robot_rx. Se um lado mudar, ninguém recebe nada e não
  // há mensagem de erro — o pacote simplesmente some.
  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);

  // Sem auto-ACK: é broadcast para vários robôs no mesmo endereço, e ninguém
  // confirma. Com ACK ligado, cada write() espera resposta e faz 15 retries,
  // travando a ponte por dezenas de ms por pacote.
  radio.setAutoAck(false);
  radio.setRetries(0, 0);

  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println(radio.isChipConnected() ? "tx_bridge | radio OK" : "tx_bridge | radio FAIL");
}

// Leitura byte a byte, usando o start byte para ressincronizar.
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

    uint8_t expected = calculateChecksum(*pkt);

    if (pkt->checksum == expected) {
      radio.write(buffer, PACKET_SIZE);

#if DEBUG_TX
      Serial.print("TX id ");
      Serial.println(pkt->robot_id);
#endif
    } else {
      // Checksum ruim quase sempre significa que perdemos o alinhamento do
      // stream. Reprocessa o buffer a partir do próximo start byte em vez de
      // descartar tudo, senão um único byte perdido derruba vários pacotes.
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
}
