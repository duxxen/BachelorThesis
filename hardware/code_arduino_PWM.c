#include "GyverPWM.h"

#define BUF_IN_SIZE   100     // размеры буферов 
#define BUF_COM_SIZE  100     // поменьше?
#define BUF_VAL_IN    100     // поменьше?

#define COMMAND_LEN       2     // длина строки команды
#define LIMITS_COMMAND    "01"  // команда установки пределов (пилы)
#define MEASURE_COMMAND   "02"  // команда проведения измерений (пила, сигнал)
#define LABEL_COMMAND     "03"  // команда снятия напряжения метки
#define MANUAL_COMMAND    "04"  // команда установки ручки

#define CHANNEL_X   A0  // Пила
#define CHANNEL_Y   A1  // Сигнал
#define CHANNEL_L   A2  // Метка   
#define CHANNEL_M   10  // Ручка

#define HIGH_VOLTAGE 5
#define MAX_PWM_VALUE 4095

uint16_t xStart { 0 };
uint16_t xEnd   { 1023 };

char buffer[BUF_IN_SIZE];
char commandIn[BUF_COM_SIZE];
char valueIn[BUF_VAL_IN];

bool inRange(float value, float from, float to) {
  return from <= value && value <= to;
}

float ranged(float value, float mn, float mx) {
  return min(max(mn, value), mx);
}

float toVoltage(uint16_t value, uint16_t RES = 1024) {
  return float(value) / RES * HIGH_VOLTAGE;
}

uint16_t toValue(float voltage, uint16_t RES = 1024) {
  return voltage / HIGH_VOLTAGE * RES;
}

uint16_t toValueInvert(float voltage, uint16_t RES = 1024) {
  return (voltage - HIGH_VOLTAGE) / -HIGH_VOLTAGE * RES;
}

void store(float x, float y) {
  Serial.print("DATA:");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print("\n");
}

void onInput() {
  size_t i = 0;
  delay(20); // обязательная задержка
  while (Serial.available() && i < BUF_IN_SIZE - 1) {
    char letter = Serial.read();
    buffer[i] = letter;
    i++;
  }
  sscanf(buffer, "%[^,],%s", commandIn, valueIn); // %s записывает любые данные в valueIn

  buffer[0] = '\0'; // Очистка буфера
}

void onCommand() {

  Serial.print("CMD: ");
  Serial.println(commandIn);
  Serial.print("VALUE: ");
  Serial.println(valueIn);

  if (strncmp(commandIn, LIMITS_COMMAND, COMMAND_LEN) == 0) {
    char v1[10], v2[10];
    sscanf(valueIn, "%[^,],%s", v1, v2);
    setLimits(atof(v1), atof(v2));
  }
  else if (strncmp(commandIn, MEASURE_COMMAND, COMMAND_LEN) == 0) {
      readSignal();
  }
  else if (strncmp(commandIn, LABEL_COMMAND, COMMAND_LEN) == 0) {
    readLabel();
  }
  else if (strncmp(commandIn, MANUAL_COMMAND, COMMAND_LEN) == 0) {
    char v1[10];
    sscanf(valueIn, "%s", v1);
    setManual(atof(v1));
  }

  commandIn[0] = '\0';
  valueIn[0] = '\0';
}

void setLimits(float umin, float umax) {
  umin = ranged(umin, 0, HIGH_VOLTAGE);
  umax = ranged(umax, 0, HIGH_VOLTAGE);
  xStart = toValue(umin);
  xEnd = toValue(umax);
}

void readSignal() {
  uint16_t sX = analogRead(CHANNEL_X);
  uint16_t sY = analogRead(CHANNEL_Y);

  if (inRange(sX, xStart, xEnd)) 
    store(toVoltage(sX), toVoltage(sY));
  else
    Serial.println("NULL");
}

void readLabel() {
  uint16_t sX = analogRead(CHANNEL_X);
  uint16_t sL = analogRead(CHANNEL_L);
  store(toVoltage(sX), toVoltage(sL));
}

void setManual(float u) {
  u = ranged(u, 0, HIGH_VOLTAGE);
  uint16_t sM = toValueInvert(u, MAX_PWM_VALUE);
  analogWrite(CHANNEL_M, sM);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(CHANNEL_X, INPUT);
  pinMode(CHANNEL_Y, INPUT);
  pinMode(CHANNEL_L, INPUT);
  pinMode(CHANNEL_M, OUTPUT);
  PWM_resolution(10, 12, FAST_PWM);

  Serial.println("MK STARTED");
}

void loop() {

  if (Serial.available()) {
    onInput();
    onCommand();
  }
}