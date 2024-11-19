#include <Wire.h>

#define BUF_IN_SIZE   100     // размеры буферов 
#define BUF_COM_SIZE  100     
#define BUF_VAL_IN    100     

#define COMMAND_LEN       2     // длина строки команды
#define LIMITS_COMMAND    "01"  // команда установки пределов (развертка)
#define MEASURE_COMMAND   "02"  // команда проведения измерений (развертка, КСВН)
#define LABEL_COMMAND     "03"  // команда проведения измерений (развертка, метка)
#define MANUAL_COMMAND    "04"  // команда установки напряжения развертки ГКЧ

#define CHANNEL_X   A0  // Развертка
#define CHANNEL_Y   A1  // КСВН
#define CHANNEL_L   A2  // Метка   

#define ADDR 0x60		// I2C Адрес ЦАП
#define HIGH_VOLTAGE 5		// Верхнее напряжение МК
#define HIGH_AMP_VOLTAGE 4.6	// Верхнее входное напряжение ОУ
#define MAX_DAC_VALUE 4095	// Разрядность ЦАП

uint16_t xStart { 0 };		// Левый предел измерения
uint16_t xEnd   { 1023 };	// Правый предел измерения

char buffer[BUF_IN_SIZE];	// Буфер вводимых данных
char commandIn[BUF_COM_SIZE];	// Буфер дешифрованной команды
char valueIn[BUF_VAL_IN];	// Буфер дешиврованных параметров

bool inRange(float value, float from, float to) {	// Функция проверки нахождения точки {value} в границах [from; to]
  return from <= value && value <= to;
}

float ranged(float value, float mn, float mx) {		// Функция ограничения значения {value} в пределы [mn; mx]
  return min(max(mn, value), mx);
}

float toVoltage(uint16_t value, uint16_t RES = 1024) { 	// Функция преобразования цифрового значения {value} в напряжение
  return float(value) / RES * HIGH_VOLTAGE;
}

uint16_t toValue(float voltage, uint16_t RES = 1024) { 	// Функция преобразования напряжения {voltage} в цифровое значение
  return voltage / HIGH_VOLTAGE * RES;
}

uint16_t toValueInvert(float voltage, uint16_t RES = 1024) {	// Инвертировання функция преобразования напряжения {voltage} в цифровое значение
  return (voltage - HIGH_VOLTAGE) / -HIGH_VOLTAGE * RES;
}

void store(float x, float y) {	// Функция передачи данных в СОМ-порт
  Serial.print("DATA:");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print("\n");
}

void onInput() {	// Функция чтения данных из СОМ-порта
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

void onCommand() {	// Функция трансляции, вызова команды

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

void setLimits(float umin, float umax) {	// Функция установки пределов измерения [umin; umax]
  umin = ranged(umin, 0, HIGH_VOLTAGE);
  umax = ranged(umax, 0, HIGH_VOLTAGE);
  xStart = toValue(umin);
  xEnd = toValue(umax);
}

void readSignal() {				// Функция измерения сигналов развертки, КСВН
  uint16_t sX = analogRead(CHANNEL_X);
  uint16_t sY = analogRead(CHANNEL_Y);

  if (inRange(sX, xStart, xEnd)) 
    store(toVoltage(sX), toVoltage(sY));
  else
    Serial.println("NULL");
}

void readLabel() {				// Функция измерения сигналов развертки, метки
  uint16_t sX = analogRead(CHANNEL_X);
  uint16_t sL = analogRead(CHANNEL_L);
  store(toVoltage(sX), toVoltage(sL));
}

void setManual(float u) {			// Функция установки напряжения развертки ГКЧ
  u = ranged(u, 0, HIGH_AMP_VOLTAGE);
  uint16_t sM = toValueInvert(u, MAX_DAC_VALUE);
  
  uint8_t cmd[] = { 0x40, sM / 16, (sM % 16) << 4 };

  Wire.beginTransmission(ADDR);
  Wire.write(cmd, 3);
  Wire.endTransmission();
}

void setup() {					// Функция инициализации МК
  Wire.begin();
  Serial.begin(9600);
  pinMode(CHANNEL_X, INPUT);
  pinMode(CHANNEL_Y, INPUT);
  pinMode(CHANNEL_L, INPUT);

  Serial.println("MK STARTED");
}

void loop() {					// Основной цикл программы

  if (Serial.available()) {
    onInput();
    onCommand();
  }
}