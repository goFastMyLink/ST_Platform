/*
  Управление платформой: MATLAB считает ОЗК и присылает углы кривошипов,
  Arduino переводит их в импульсы PCA9685 и плавно ведёт сервы к цели.

  Протокол (115200, строки заканчиваются '\n'):
    A a1 a2 a3   - углы кривошипов, град (как в MATLAB: 0 - горизонтально, + вниз;
                   рабочий ход только ВВЕРХ: от A_MAX (у горизонтали) до A_MIN (-65°))
                   ответ: OK a1 a2 a3   (если угол обрезан пределами - CLIP a1 a2 a3)
    Z            - все в нейтраль A_NEUTRAL   ответ: OK a a a
    F            - обратная связь         ответ: FB adc1 adc2 adc3
    M            - MPU на платформе       ответ: MPU roll pitch g n
                   roll, pitch - град (оси самого MPU), g - модуль ускорения в g (в покое ~1.00),
                   n - сколько раз MPU пропадал и был заново инициализирован с момента старта
                   (если MPU сейчас не отвечает - ERR mpu)
  Серво 1, 2, 3 - приводы в шарнирах B1, B3, B5 по схеме (A1, A3, A5 в MATLAB).

  MPU6050 (GY-521) и PCA9685 - на одной шине I2C (у Mega пины SDA/SCL возле AREF
  и пины 20/21 - это одна и та же шина). Адреса разные: MPU 0x68, PCA 0x40.
  При старте ~1 с платформу НЕ трогать: калибруется нуль гироскопа.
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ---- калибровка (servo_calibration, 06.10.2026: горизонталь и вертикаль по уровню) ----
const uint8_t CH[3]     = {0, 1, 2};
const uint8_t FB[3]     = {A0, A1, A2};
const float   US0[3]    = {937.5, 888.7, 913.1};   // импульс при a = 0 (кривошип горизонтален)
const float   US_DEG[3] = {-7.05, -7.60, -6.51};   // мкс на градус: (вертикаль - горизонталь) / -90
const float   FREQ      = 50.0;

// ---- ограничения ----
const float A_MIN = -65.0, A_MAX = -2.0;  // град: вниз от горизонтали упор; около -80 мёртвая точка
                                          // (кривошип ложится на линию тяги, test_3servo), берём запас
const float A_NEUTRAL = -22.0;           // рабочая нейтраль: кривошипы на 22° выше горизонтали
const float US_MIN = 500.0, US_MAX = 2500.0;
const float RATE  = 60.0;                 // макс. скорость кривошипа, град/с

// ---- MPU6050 ----
const uint8_t MPU      = 0x68;     // AD0 на GND (на 3.3 В - 0x69)
const float   ACC_LSB  = 4096.0;   // ±8 g
const float   GYR_LSB  = 65.5;     // ±500 °/с
const float   ALPHA    = 0.98;     // комплементарный фильтр: доля гироскопа (τ ≈ 0.25 с при 200 Гц)
const unsigned long MPU_US = 5000; // период опроса, мкс (200 Гц)
int16_t raw[7];                    // ax ay az t gx gy gz
float gBias[3] = {0, 0, 0};
float roll = 0, pitch = 0, gNorm = 1;
bool  mpuOk = false;
bool  mpuFresh = true;             // после (пере)инициализации углы берём заново с акселерометра
unsigned int mpuResets = 0;
unsigned long tMpu = 0;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

bool mpuRead() {                   // все 14 байт за один запрос: акселерометр, температура, гироскоп
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  for (int i = 0; i < 7; i++) {
    int16_t hi = Wire.read();      // отдельным оператором: порядок вычисления в одном выражении не определён
    raw[i] = (hi << 8) | Wire.read();
  }
  return true;
}

void accAngles(float& r, float& p) {
  float ax = raw[0] / ACC_LSB, ay = raw[1] / ACC_LSB, az = raw[2] / ACC_LSB;
  r = atan2(ay, az) * RAD_TO_DEG;
  p = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  gNorm = sqrt(ax * ax + ay * ay + az * az);
}

uint8_t mpuInit() {                // возвращает WHO_AM_I (0x68 у оригинала, у клонов бывает 0x70/0x72/0x98), 0 - нет ответа
  mpuWrite(0x6B, 0x00); delay(100);  // разбудить
  mpuWrite(0x1A, 0x03);              // цифровой ФНЧ ~44 Гц
  mpuWrite(0x1B, 0x08);              // гироскоп ±500 °/с
  mpuWrite(0x1C, 0x10);              // акселерометр ±8 g
  Wire.beginTransmission(MPU); Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(MPU, (uint8_t)1) != 1) return 0;
  return Wire.read();
}

void mpuCalib() {                  // нуль гироскопа (платформа неподвижна) + начальные углы по акселерометру
  long sum[3] = {0, 0, 0}; int n = 0;
  for (int i = 0; i < 400; i++) {
    if (mpuRead()) { for (int k = 0; k < 3; k++) sum[k] += raw[4 + k]; n++; }
    delay(2);
  }
  for (int k = 0; k < 3; k++) gBias[k] = n ? (float)sum[k] / n : 0;
  mpuOk = mpuRead();
  if (mpuOk) accAngles(roll, pitch);
  if (mpuOk && gNorm < 0.3) mpuOk = false;
  mpuFresh = !mpuOk;
  tMpu = micros();
}

void mpuUpdate() {
  unsigned long now = micros();
  float dt = (now - tMpu) * 1e-6;  // реальный шаг, а не константа
  tMpu = now;
  if (dt > 0.05) dt = 0.05;
  mpuOk = mpuRead();
  if (!mpuOk) return;
  float rA, pA; accAngles(rA, pA);
  if (gNorm < 0.3) {               // все нули: MPU перезагрузился (просадка питания/помеха) и уснул
    mpuOk = false; mpuFresh = true; mpuResets++;
    mpuInit();                     // разбудить и заново настроить (~0.1 с); нуль гироскопа сохраняется
    tMpu = micros();
    return;
  }
  if (mpuFresh) { roll = rA; pitch = pA; mpuFresh = false; return; }
  float gx = (raw[4] - gBias[0]) / GYR_LSB, gy = (raw[5] - gBias[1]) / GYR_LSB;
  float a = (fabs(gNorm - 1.0) < 0.15) ? ALPHA : 1.0;   // при рывках акселерометру не верим
  roll  = a * (roll  + gx * dt) + (1 - a) * rA;
  pitch = a * (pitch + gy * dt) + (1 - a) * pA;
}

float cur[3] = {A_NEUTRAL, A_NEUTRAL, A_NEUTRAL}, tgt[3] = {A_NEUTRAL, A_NEUTRAL, A_NEUTRAL};
char line[64]; uint8_t len = 0;
unsigned long tPrev = 0;

void writeAngle(int k, float a) {
  float us = constrain(US0[k] + US_DEG[k] * a, US_MIN, US_MAX);
  pca.setPWM(CH[k], 0, (int)(us * FREQ * 4096.0 / 1e6 + 0.5));
}

void reply(const char* tag, const float* a) {
  Serial.print(tag);
  for (int k = 0; k < 3; k++) { Serial.print(' '); Serial.print(a[k], 2); }
  Serial.println();
}

void handle(char* s) {
  char* tok = strtok(s, " \t\r");
  if (!tok) return;
  if (tok[0] == 'A') {
    float a[3]; bool clip = false;
    for (int k = 0; k < 3; k++) {
      tok = strtok(NULL, " \t\r");
      if (!tok) { Serial.println("ERR need 3 angles"); return; }
      a[k] = atof(tok);
      if (a[k] < A_MIN || a[k] > A_MAX) { a[k] = constrain(a[k], A_MIN, A_MAX); clip = true; }
    }
    for (int k = 0; k < 3; k++) tgt[k] = a[k];
    reply(clip ? "CLIP" : "OK", tgt);
  } else if (tok[0] == 'Z') {
    for (int k = 0; k < 3; k++) tgt[k] = A_NEUTRAL;
    reply("OK", tgt);
  } else if (tok[0] == 'M') {
    if (!mpuOk) { Serial.println("ERR mpu"); return; }
    Serial.print("MPU "); Serial.print(roll, 2);
    Serial.print(' ');    Serial.print(pitch, 2);
    Serial.print(' ');    Serial.print(gNorm, 3);
    Serial.print(' ');    Serial.println(mpuResets);
  } else if (tok[0] == 'F') {
    Serial.print("FB");
    for (int k = 0; k < 3; k++) { Serial.print(' '); Serial.print(analogRead(FB[k])); }
    Serial.println();
  } else {
    Serial.println("ERR unknown");
  }
}

void setup() {
  Serial.begin(115200);
  pca.begin();
  pca.setOscillatorFrequency(25000000);
  pca.setPWMFreq(FREQ);
  delay(10);
  for (int k = 0; k < 3; k++) writeAngle(k, A_NEUTRAL);   // старт сразу в нейтраль, не на упор
  Wire.setWireTimeout(3000, true);  // если шина I2C зависнет (помеха по длинным проводам) - не вешать скетч
  uint8_t who = mpuInit();
  delay(600);                       // дать сервам доехать и платформе успокоиться
  mpuCalib();
  tPrev = millis();
  // одна строка: platform_connect ждёт первую строку со словом READY
  Serial.print("READY MPU "); Serial.print(mpuOk ? "ok" : "FAIL");
  Serial.print(" who=0x"); Serial.println(who, HEX);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { line[len] = 0; handle(line); len = 0; }
    else if (len < sizeof(line) - 1) line[len++] = c;
  }
  if (micros() - tMpu >= MPU_US) mpuUpdate();
  unsigned long now = millis();
  if (now - tPrev >= 20) {                       // плавное движение, 50 Гц
    float step = RATE * (now - tPrev) / 1000.0;
    tPrev = now;
    for (int k = 0; k < 3; k++) {
      float d = tgt[k] - cur[k];
      if (d > step) d = step; else if (d < -step) d = -step;
      if (d != 0) { cur[k] += d; writeAngle(k, cur[k]); }
    }
  }
}
