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

// Ligável por -DDEBUG_TX=1 na compilação, sem editar o arquivo. Com isso a ponte
// confirma pela serial cada pacote que ela repassou ao rádio, que é o que separa
// "a ponte não recebeu" de "a ponte recebeu e mandou".
#ifndef DEBUG_TX
#define DEBUG_TX 0
#endif

RF24 radio(6, 10);

// Um endereço por robô. Ver o bloco do auto-ACK no setup() para o porquê.
// Mesma tabela no `robot_rx.ino` e no `tx_probe.ino`. Mudou aqui, mude nos três.
constexpr uint8_t MAX_ROBOS = 2;
const byte ENDERECOS[MAX_ROBOS][6] = {"VSS00", "VSS01"};

// Trocar o pipe custa algumas escritas SPI, então só troca quando o destino
// muda. A 30 Hz alternando entre dois robôs isso é uma troca por pacote, que o
// loop aguenta; guardar o último evita a troca redundante quando só um joga.
int32_t destino_atual = -1;

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

  // ============================================================================
  //  NÃO DESLIGUE O AUTO-ACK. Medido em 10/08/2026, robô a 1 m da ponte, 120
  //  pacotes a 30 Hz, contando os aceitos no log do próprio robô:
  //
  //      ACK off, retries 0,0  ->    0,0%     (era esta a configuração daqui)
  //      ACK off, retries 5,5  ->    0,0%
  //      ACK on,  retries 0,0  ->   87,5%
  //      ACK on,  retries 5,5  ->  100,0%
  //
  //  Zero, não "ruim". Com auto-ACK desligado este link não entrega um único
  //  pacote — e nem a ponte nem o robô registram erro, porque `write()` sem ACK
  //  devolve `true` assim que o pacote sai do FIFO, sem prova de que alguém
  //  ouviu. O sintoma é robô parado com os dois logs limpos.
  //
  //  O comentário antigo aqui dizia que ACK travaria a ponte com 15 retries.
  //  A parte verdadeira era o 15: o default. Com 5 o custo é o medido acima.
  // ============================================================================
  radio.setAutoAck(true);

  // 5 retentativas a 1,5 ms. Recupera as perdas (87,5% -> 100%) sem o default de
  // 15, que trava o loop por ~60 ms por pacote quando um robô está desligado.
  radio.setRetries(5, 5);

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
      // Um id fora da tabela indexaria ENDERECOS fora do array e mandaria o
      // pacote para um endereço de lixo. Descarta, e diz por quê.
      if (pkt->robot_id < 0 || pkt->robot_id >= MAX_ROBOS) {
#if DEBUG_TX
        Serial.print("ID FORA DA TABELA: ");
        Serial.println(pkt->robot_id);
#endif
        bufferIndex = 0;
        continue;
      }

      if (pkt->robot_id != destino_atual) {
        radio.openWritingPipe(ENDERECOS[pkt->robot_id]);
        destino_atual = pkt->robot_id;
      }

      // Com ACK ligado isto devolve `true` só se o robô confirmou em hardware —
      // então aqui existe uma medida de entrega real, que sem ACK não existia.
      bool entregue = radio.write(buffer, PACKET_SIZE);

#if DEBUG_TX
      Serial.print("TX id ");
      Serial.print(pkt->robot_id);
      Serial.println(entregue ? " ACK" : " SEM ACK");
#else
      (void)entregue;
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
