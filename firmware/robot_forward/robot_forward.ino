// robot_forward — as duas rodas para frente, constante, sem rádio e sem serial
//
// O sketch mais burro possível: liga o driver e manda as duas rodas para frente
// a 0,15 na mesma escala do resto do projeto (Motor1/Motor2 em [-1,0 .. 1,0],
// como chegam no `robot_rx`). Serve para ver o robô andar sozinho na bancada,
// medir `wheel_speed_max` cronometrando um metro, ou conferir polaridade.
//
// ATENÇÃO: ele começa a andar assim que a placa liga, e nunca para. Não há
// timeout de segurança, ao contrário do `robot_rx` (que para sem rádio) e do
// `robot_selftest` (que para sem tecla). Segure o robô ou suspenda as rodas.
//
// As funções motorA/motorB são cópia literal das do `robot_rx`, inclusive o
// PWM_DEADZONE — de propósito, para que o comportamento seja o mesmo do
// firmware de jogo. Se mudar lá, mude aqui.
//
// Wiring (idêntico ao robot_rx, menos o rádio, que não é usado):
//   TB6612FNG: PWMA=D5, AIN1=D4, AIN2=D3, PWMB=D9, BIN1=D7, BIN2=D8, STBY=D2
//
// Uso:
//   arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 firmware/robot_forward
//   arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328 \
//                      firmware/robot_forward

#include <stdint.h>

// ── Motor pins ───────────────────────────────────────────
#define PWMA 5
#define AIN1 4
#define AIN2 3
#define PWMB 9
#define BIN1 7
#define BIN2 8
#define STBY 2

// Abaixo deste PWM o motor só chia e esquenta sem girar. Igual ao robot_rx.
constexpr int PWM_DEADZONE = 25;

// ── Velocidade ───────────────────────────────────────────
// Mesma escala do campo Motor1/Motor2 do pacote de rádio: 1,0 = PWM 255.
// 0,15 vira PWM 38 — acima do deadzone (25), mas por pouco. Se este robô não
// sair do lugar, rode a rampa do `robot_selftest` (tecla `p`) para descobrir o
// PWM mínimo real dele antes de concluir que é problema de motor.
constexpr float VELOCIDADE = 0.15f;

// ── Motor functions (espelham robot_rx.ino) ──────────────
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

void enableMotors() { digitalWrite(STBY, HIGH); }

// ── Setup / loop ─────────────────────────────────────────
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

  const int pwm = static_cast<int>(VELOCIDADE * 255.0f);

  // Como o comando nunca muda, escrever uma vez basta: os pinos de direção e o
  // PWM ficam segurados pelo hardware. O loop fica vazio de propósito.
  motorA(pwm);
  motorB(pwm);

  Serial.print(F("robot_forward | velocidade "));
  Serial.print(VELOCIDADE, 2);
  Serial.print(F(" -> PWM "));
  Serial.println(pwm);

  if (pwm < PWM_DEADZONE) {
    Serial.println(F("AVISO: PWM abaixo do deadzone, as rodas vao ficar paradas"));
  }
}

void loop() {
  // Nada a fazer: o comando é constante e já foi aplicado no setup().
}
