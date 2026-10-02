/*
  Управление платформой: MATLAB считает ОЗК и присылает углы кривошипов,
  Arduino переводит их в импульсы PCA9685 и плавно ведёт сервы к цели.

  Протокол (115200, строки заканчиваются '\n'):
    A a1 a2 a3   - углы кривошипов, град (как в MATLAB: 0 - горизонтально, + вниз)
                   ответ: OK a1 a2 a3   (если угол обрезан пределами - CLIP a1 a2 a3)
    Z            - все в ноль             ответ: OK 0 0 0
    F            - обратная связь         ответ: FB adc1 adc2 adc3
  Серво 1, 2, 3 - приводы в шарнирах B1, B3, B5 по схеме (A1, A3, A5 в MATLAB).
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ---- калибровка (servo_calibration, 02.10.2026) ----
const uint8_t CH[3]     = {0, 1, 2};
const uint8_t FB[3]     = {A0, A1, A2};
const float   US0[3]    = {864.3, 668.9, 913.1};   // импульс при a = 0 (кривошип горизонтален)
const float   US_DEG[3] = {-8.0, -8.0, -8.0};      // мкс на градус - ПРЕДВАРИТЕЛЬНО, уточнить
const float   FREQ      = 50.0;

// ---- ограничения ----
const float A_MIN = -30.0, A_MAX = 20.0;  // град: вниз (+) запас мал у серво 2 (ноль ~669 мкс)
const float US_MIN = 500.0, US_MAX = 2500.0;
const float RATE  = 60.0;                 // макс. скорость кривошипа, град/с

float cur[3] = {0, 0, 0}, tgt[3] = {0, 0, 0};
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
    for (int k = 0; k < 3; k++) tgt[k] = 0;
    reply("OK", tgt);
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
  for (int k = 0; k < 3; k++) writeAngle(k, 0);
  tPrev = millis();
  Serial.println("READY");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { line[len] = 0; handle(line); len = 0; }
    else if (len < sizeof(line) - 1) line[len++] = c;
  }
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
