// robot_selftest — bancada de teste do robô VSS, sem rádio
//
// Mesmo hardware e mesma camada de motor do `robot_rx`, mas os comandos nascem
// aqui dentro em vez de chegarem pelo nRF24. Serve para validar um robô novo
// antes de haver rádio: fiação, polaridade dos motores, qual motor é o A, e a
// partir de que PWM cada roda realmente sai do lugar.
//
// As funções motorA/motorB abaixo são cópia literal das do `robot_rx`
// (inclusive PWM_DEADZONE). Isso é de propósito: o que você aferir aqui é o
// que o firmware de jogo vai fazer. Se mudar lá, mude aqui.
//
// Wiring (idêntico ao robot_rx, menos o rádio, que não é usado):
//   TB6612FNG: PWMA=D5, AIN1=D4, AIN2=D3, PWMB=D9, BIN1=D7, BIN2=D8, STBY=D2
//
// Uso:
//   arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 firmware/robot_selftest
//   arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328 \
//                      firmware/robot_selftest
//   arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200
//
// No monitor serial, tecle `h` para o menu. Comece com `r` (roteiro de
// aferição) com o robô suspenso, rodas no ar.

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
// O comando `p` mede o valor real deste robô; se der bem diferente de 25,
// esse é um número para corrigir nos dois arquivos.
constexpr int PWM_DEADZONE = 25;

// ── Segurança ────────────────────────────────────────────
// No modo manual o robô para sozinho depois deste tempo sem tecla nova. Sem
// isso, um `w` esquecido manda o robô para o chão enquanto você olha a tela.
// Segure a tecla (auto-repeat do terminal) para andar continuamente.
constexpr unsigned long HOLD_TIMEOUT_MS = 1500;

// Velocidade do modo manual, ajustável com + e -.
int manualSpeed = 150;
constexpr int SPEED_STEP = 25;

unsigned long lastCommandTime = 0;
bool moving = false;

// ── Motor functions (espelham robot_rx.ino) ──────────────
// `useDeadzone=false` existe só para a rampa do comando `p`, que precisa
// justamente varrer a faixa que o firmware de jogo descarta.
void motorA(int speed, bool useDeadzone = true) {
  if (useDeadzone && abs(speed) < PWM_DEADZONE) speed = 0;

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

void motorB(int speed, bool useDeadzone = true) {
  if (useDeadzone && abs(speed) < PWM_DEADZONE) speed = 0;

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
  moving = false;
}

void enableMotors() { digitalWrite(STBY, HIGH); }

void disableMotors() { digitalWrite(STBY, LOW); }

// Todo comando de movimento passa por aqui, para que o timeout de segurança
// valha para todos e o log fique uniforme.
void drive(int a, int b, const __FlashStringHelper *label) {
  motorA(a);
  motorB(b);

  moving = (a != 0 || b != 0);
  lastCommandTime = millis();

  Serial.print(F("> "));
  Serial.print(label);
  Serial.print(F("  A="));
  Serial.print(a);
  Serial.print(F(" B="));
  Serial.println(b);
}

// ── Roteiro de aferição ──────────────────────────────────
// Cada passo diz antes o que você deveria ver. É essa comparação que revela
// fio trocado: se o texto diz "roda A para frente" e gira a B, ou gira ao
// contrário, o problema está na fiação, não no software.
struct Step {
  int a;
  int b;
  unsigned int ms;
  const __FlashStringHelper *what;
};

// As descrições ficam em PROGMEM. Em RAM, só os textos desta tabela já comiam
// um terço dos 2 KB do Nano — e o `F()` não pode ser usado em inicializador
// global, daí a declaração separada. Os passos parados compartilham a mesma
// string de propósito.
#define FLASH_STR(name, text) \
  const char name[] PROGMEM = text; \
  const __FlashStringHelper *const name##_F = reinterpret_cast<const __FlashStringHelper *>(name)

FLASH_STR(TXT_IDLE0, "parado (confira que nada gira)");
FLASH_STR(TXT_IDLE,  "parado");
FLASH_STR(TXT_A_FWD, "SO a roda A, para FRENTE");
FLASH_STR(TXT_A_REV, "SO a roda A, para TRAS");
FLASH_STR(TXT_B_FWD, "SO a roda B, para FRENTE");
FLASH_STR(TXT_B_REV, "SO a roda B, para TRAS");
FLASH_STR(TXT_FWD,   "as duas para FRENTE (robo anda reto)");
FLASH_STR(TXT_REV,   "as duas para TRAS");
FLASH_STR(TXT_SPIN1, "giro no proprio eixo, sentido 1");
FLASH_STR(TXT_SPIN2, "giro no proprio eixo, sentido 2");
FLASH_STR(TXT_SLOW,  "as duas para frente, PWM baixo (80)");
FLASH_STR(TXT_END,   "fim");

const Step SEQUENCE[] = {
  {   0,    0,  500, TXT_IDLE0_F },
  { 180,    0, 1500, TXT_A_FWD_F },
  {   0,    0,  600, TXT_IDLE_F },
  {-180,    0, 1500, TXT_A_REV_F },
  {   0,    0,  600, TXT_IDLE_F },
  {   0,  180, 1500, TXT_B_FWD_F },
  {   0,    0,  600, TXT_IDLE_F },
  {   0, -180, 1500, TXT_B_REV_F },
  {   0,    0,  600, TXT_IDLE_F },
  { 180,  180, 1500, TXT_FWD_F },
  {   0,    0,  600, TXT_IDLE_F },
  {-180, -180, 1500, TXT_REV_F },
  {   0,    0,  600, TXT_IDLE_F },
  { 180, -180, 1200, TXT_SPIN1_F },
  {   0,    0,  600, TXT_IDLE_F },
  {-180,  180, 1200, TXT_SPIN2_F },
  {   0,    0,  600, TXT_IDLE_F },
  {  80,   80, 1500, TXT_SLOW_F },
  {   0,    0,    0, TXT_END_F },
};

constexpr size_t SEQUENCE_LEN = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);

// Espera bloqueante que continua ouvindo o serial: qualquer tecla aborta o
// roteiro. Sem isso, um robô com fio trocado fica 20 s se debatendo até o fim.
bool waitOrAbort(unsigned long ms) {
  unsigned long start = millis();

  while (millis() - start < ms) {
    if (Serial.available()) {
      Serial.read();
      return false;
    }
  }

  return true;
}

void runSequence() {
  Serial.println();
  Serial.println(F("=== roteiro de afericao (qualquer tecla aborta) ==="));
  Serial.println(F("Suspenda o robo: as rodas precisam girar no ar."));

  for (size_t i = 0; i < SEQUENCE_LEN; i++) {
    const Step &s = SEQUENCE[i];

    Serial.print(F("["));
    Serial.print(i + 1);
    Serial.print(F("/"));
    Serial.print(SEQUENCE_LEN);
    Serial.print(F("] "));
    Serial.println(s.what);

    motorA(s.a);
    motorB(s.b);

    if (!waitOrAbort(s.ms)) {
      stopAll();
      Serial.println(F("=== abortado ==="));
      return;
    }
  }

  stopAll();
  Serial.println(F("=== roteiro terminado ==="));
}

// ── Rampa: acha o PWM minimo real de cada roda ───────────
// Sobe o PWM devagar e imprime o valor. Anote em que numero a roda comeca a
// girar de fato — esse e o PWM_DEADZONE honesto deste robo, e o piso da faixa
// util para calibrar `wheel_speed_max` depois.
void runRamp(char which) {
  Serial.println();
  Serial.print(F("=== rampa da roda "));
  Serial.print(which);
  Serial.println(F(" (qualquer tecla aborta) ==="));
  Serial.println(F("Anote o PWM em que a roda comeca a girar."));

  for (int pwm = 0; pwm <= 255; pwm += 5) {
    if (which == 'A') {
      motorA(pwm, false);
    } else {
      motorB(pwm, false);
    }

    Serial.print(F("PWM = "));
    Serial.println(pwm);

    if (!waitOrAbort(400)) {
      stopAll();
      Serial.println(F("=== abortado ==="));
      return;
    }
  }

  stopAll();
  Serial.println(F("=== rampa terminada ==="));
}

// ── Menu ─────────────────────────────────────────────────
void printHelp() {
  Serial.println();
  Serial.println(F("── robot_selftest ─────────────────────────────"));
  Serial.println(F(" r  roteiro de afericao completo"));
  Serial.println(F(" p  rampa da roda A (acha o PWM minimo)"));
  Serial.println(F(" o  rampa da roda B"));
  Serial.println();
  Serial.println(F(" w  frente        s  re"));
  Serial.println(F(" a  gira esquerda d  gira direita"));
  Serial.println(F(" 1  so a roda A   2  so a roda B"));
  Serial.println(F(" x  parar         espaco  parar"));
  Serial.println(F(" +  mais rapido   -  mais devagar"));
  Serial.println(F(" q  standby do driver (motores desligados)"));
  Serial.println(F(" e  sai do standby"));
  Serial.println(F(" h  esta ajuda"));
  Serial.println();
  Serial.print(F("velocidade atual: "));
  Serial.print(manualSpeed);
  Serial.print(F("   parada automatica: "));
  Serial.print(HOLD_TIMEOUT_MS);
  Serial.println(F(" ms sem tecla"));
  Serial.println(F("───────────────────────────────────────────────"));
}

void handleKey(char c) {
  switch (c) {
    case 'r': runSequence(); break;
    case 'p': runRamp('A'); break;
    case 'o': runRamp('B'); break;

    case 'w': drive( manualSpeed,  manualSpeed, F("frente")); break;
    case 's': drive(-manualSpeed, -manualSpeed, F("re")); break;
    case 'a': drive(-manualSpeed,  manualSpeed, F("gira esquerda")); break;
    case 'd': drive( manualSpeed, -manualSpeed, F("gira direita")); break;
    case '1': drive( manualSpeed,            0, F("so roda A")); break;
    case '2': drive(           0,  manualSpeed, F("so roda B")); break;

    case 'x':
    case ' ':
      stopAll();
      Serial.println(F("> parado"));
      break;

    case '+':
    case '=':
      manualSpeed = constrain(manualSpeed + SPEED_STEP, 0, 255);
      Serial.print(F("velocidade: "));
      Serial.println(manualSpeed);
      break;

    case '-':
    case '_':
      manualSpeed = constrain(manualSpeed - SPEED_STEP, 0, 255);
      Serial.print(F("velocidade: "));
      Serial.println(manualSpeed);
      break;

    case 'q':
      stopAll();
      disableMotors();
      Serial.println(F("> STBY baixo: driver desligado"));
      break;

    case 'e':
      enableMotors();
      Serial.println(F("> STBY alto: driver ligado"));
      break;

    case 'h':
    case '?':
      printHelp();
      break;

    default:
      break;  // \r, \n e ruído de terminal caem aqui de propósito
  }
}

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
  stopAll();

  Serial.println();
  Serial.println(F("robot_selftest | sem radio, comandos pelo serial"));
  printHelp();
}

void loop() {
  if (moving && millis() - lastCommandTime > HOLD_TIMEOUT_MS) {
    stopAll();
    Serial.println(F("> parada automatica (sem tecla)"));
  }

  if (Serial.available()) {
    handleKey(Serial.read());
  }
}
