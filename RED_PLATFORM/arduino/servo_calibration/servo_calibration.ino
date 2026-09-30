/*
  Калибровка нуля серво FB5118M через PCA9685.
  Библиотека: Adafruit PWM Servo Driver (Менеджер библиотек Arduino IDE).
  Подключение PCA9685 к Arduino Uno/Nano: SDA -> A4, SCL -> A5, VCC -> 5V, GND -> GND.
  Серво - на каналы PCA9685 из массива CH (поменяй, если подключены к другим).
  Серво 1, 2, 3 - это приводы в шарнирах B1, B3, B5 по схеме платформы
  (в MATLAB-коде эти же шарниры называются A1, A3, A5). К пинам Arduino A1..A5 отношения не имеют.

  Монитор порта: 115200, "Нет конца строки" / "NL" - не важно.
  Команды:
    1 / 2 / 3  - выбрать серво (шарниры B1, B3, B5 по схеме)
    + / -      - импульс +1 / -1 отсчёт  (~4.9 мкс)
    ] / [      - импульс +10 / -10 отсчётов (~49 мкс)
    0          - вернуть выбранную серво в 1500 мкс
    p          - напечатать отсчёты всех трёх серво
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

const uint8_t CH[3] = {0, 1, 2};         // номера каналов PCA9685 (0..15) для серво в B1, B3, B5
const float FREQ = 50.0;                 // Гц - та же частота, что cal.freq в servoToPCA.m
const int T_MIN = 102, T_MAX = 512;      // ~500..2500 мкс - защита от упора
int ticks[3] = {307, 307, 307};          // 307 отсчётов ~ 1500 мкс
int sel = 0;

float toUs(int t) { return t * 1e6 / (FREQ * 4096.0); }

void apply(int k) {
  ticks[k] = constrain(ticks[k], T_MIN, T_MAX);
  pca.setPWM(CH[k], 0, ticks[k]);
}

void printAll() {
  for (int k = 0; k < 3; k++) {
    Serial.print(k == sel ? "> " : "  ");
    Serial.print("серво "); Serial.print(k + 1);
    Serial.print(": "); Serial.print(ticks[k]);
    Serial.print(" отсч. = "); Serial.print(toUs(ticks[k]), 1); Serial.println(" мкс");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pca.begin();
  pca.setOscillatorFrequency(25000000);  // у клонов бывает 24-27 МГц, на калибровку не влияет
  pca.setPWMFreq(FREQ);
  delay(10);
  for (int k = 0; k < 3; k++) apply(k);
  Serial.println("Калибровка нуля. Команды: 1 2 3, + -, ] [, 0, p");
  printAll();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case '1': case '2': case '3': sel = c - '1'; break;
    case '+': ticks[sel] += 1;  apply(sel); break;
    case '-': ticks[sel] -= 1;  apply(sel); break;
    case ']': ticks[sel] += 10; apply(sel); break;
    case '[': ticks[sel] -= 10; apply(sel); break;
    case '0': ticks[sel] = 307; apply(sel); break;
    case 'p': break;
    default: return;                     // переводы строк и прочее - игнор
  }
  printAll();
}
