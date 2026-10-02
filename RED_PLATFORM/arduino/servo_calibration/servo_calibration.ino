/*
  Калибровка нуля серво FB5118M через PCA9685.
  Библиотека: Adafruit PWM Servo Driver (Менеджер библиотек Arduino IDE).
  Подключение PCA9685 к Arduino Uno/Nano: SDA -> A4, SCL -> A5, VCC -> 5V, GND -> GND.
  Серво - на каналы PCA9685 из массива CH (поменяй, если подключены к другим).
  Серво 1, 2, 3 - это приводы в шарнирах B1, B3, B5 по схеме платформы
  (в MATLAB-коде эти же шарниры называются A1, A3, A5). К пинам Arduino A1..A5 отношения не имеют.

  Обратная связь FB5118M (4-й провод) -> аналоговые пины FB (по умолчанию A0, A1, A2).
  Напряжение на нём пропорционально фактическому углу вала: так видно, куда вал
  реально встал, и не упирается ли серва (командуешь одно, а вал стоит).

  Монитор порта: 115200, "Нет конца строки" / "NL" - не важно.
  Команды:
    1 / 2 / 3  - выбрать серво (шарниры B1, B3, B5 по схеме)
    + / -      - импульс +1 / -1 отсчёт  (~4.9 мкс)
    ] / [      - импульс +10 / -10 отсчётов (~49 мкс)
    0          - вернуть выбранную серво в 1500 мкс
    p          - напечатать отсчёты всех трёх серво и их обратную связь
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

const uint8_t CH[3] = {0, 1, 2};         // номера каналов PCA9685 (0..15) для серво в B1, B3, B5
const float FREQ = 50.0;                 // Гц - та же частота, что cal.freq в servoToPCA.m
const int T_MIN = 184, T_MAX = 430;      // ~900..2100 мкс - рабочий диапазон FB5118M, дальше вал не идёт
const uint8_t FB[3] = {A0, A1, A2};      // провод обратной связи серво в B1, B3, B5
int ticks[3] = {307, 307, 307};          // 307 отсчётов ~ 1500 мкс
int sel = 0;

float toUs(int t) { return t * 1e6 / (FREQ * 4096.0); }

void apply(int k) {
  ticks[k] = constrain(ticks[k], T_MIN, T_MAX);
  pca.setPWM(CH[k], 0, ticks[k]);
}

// Среднее из 16 замеров АЦП (Arduino Uno/Nano: 0..1023 на 0..5 В)
int readFB(int k) {
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(FB[k]);
  return sum / 16;
}

void printAll() {
  delay(300);                            // дать валу доехать перед замером
  for (int k = 0; k < 3; k++) {
    int adc = readFB(k);
    Serial.print(k == sel ? "> " : "  ");
    Serial.print("серво "); Serial.print(k + 1);
    Serial.print(": "); Serial.print(ticks[k]);
    Serial.print(" отсч. = "); Serial.print(toUs(ticks[k]), 1); Serial.print(" мкс");
    Serial.print(" | обр. связь: "); Serial.print(adc);
    Serial.print(" АЦП = "); Serial.print(adc * 5.0 / 1023.0, 3); Serial.println(" В");
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
