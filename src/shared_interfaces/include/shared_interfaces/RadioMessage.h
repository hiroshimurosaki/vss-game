#ifndef RADIO_MESSAGE_H
#define RADIO_MESSAGE_H

#include <cstddef>
#include <cstdint>

namespace shared_interfaces {

// Layout do pacote trocado entre o ROS e o Arduino (TX bridge -> rádio -> robô).
//
// ATENÇÃO: esta struct precisa bater byte a byte com a `Message` do firmware
// (firmware/tx_bridge e firmware/robot_rx). São 14 bytes:
//
//   startByte (1) + velMotor1 (4) + velMotor2 (4) + robotId (4) + checksum (1)
//
// As velocidades são float (não double) e normalizadas em [-1.0, 1.0]; o firmware
// multiplica por 255 para virar PWM.

#pragma pack(push, 1)

typedef struct {
    uint8_t startByte; // marca o início de um pacote no stream serial
    float velMotor1;   // roda esquerda, [-1.0, 1.0]
    float velMotor2;   // roda direita, [-1.0, 1.0]
    int32_t robotId;   // qual robô deve obedecer este pacote
    uint8_t checksum;  // XOR de velMotor1..robotId (não inclui startByte)
} RadioMessage;

#pragma pack(pop)

static_assert(sizeof(RadioMessage) == 14,
    "RadioMessage precisa ter 14 bytes para bater com o firmware");

} // namespace shared_interfaces

#endif
