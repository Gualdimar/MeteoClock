/*
  Скетч к проекту "Домашняя метеостанция"
  Страница проекта (схемы, описания): https://alexgyver.ru/meteoclock/
  Исходники на GitHub: https://github.com/AlexGyver/MeteoClock
  Нравится, как написан и закомментирован код? Поддержи автора! https://alexgyver.ru/support_alex/
  Автор: AlexGyver Technologies, 2018
  http://AlexGyver.ru/
*/

/*
  Время и дата устанавливаются атвоматически при загрузке прошивки (такие как на компьютере)
  График всех величин за час и за сутки (усреднённые за каждый час)
  В модуле реального времени стоит батарейка, которая продолжает отсчёт времени после выключения/сброса питания
  Как настроить время на часах. У нас есть возможность автоматически установить время на время загрузки прошивки, поэтому:
	- Ставим настройку RESET_CLOCK на 1
  - Прошиваемся
  - Сразу ставим RESET_CLOCK 0
  - И прошиваемся ещё раз
  - Всё
*/

/* Версия 1.5
  - Добавлено управление яркостью
  - Яркость дисплея и светодиода СО2 меняется на максимальную и минимальную в зависимости от сигнала с фоторезистора
  Подключите датчик (фоторезистор) по схеме. Теперь на экране отладки справа на второй строчке появится величина сигнала
  с фоторезистора.
*/

// ------------------------- НАСТРОЙКИ --------------------
#define RESET_CLOCK 0       // сброс часов на время загрузки прошивки (для модуля с несъёмной батарейкой). Не забудь поставить 0 и прошить ещё раз!
#define SENS_TIME 10000     // время обновления показаний сенсоров на экране, миллисекунд
#define LED_MODE 0          // тип RGB светодиода: 0 - главный катод, 1 - главный анод

// управление яркостью
#define BRIGHT_CONTROL 2      // 0 - запретить, яркость всегда будет макс; 1 - управление яркостью по пороговому значенипю(BRIGHT_THRESHOLD); 2 - динамическое управление яркостью
#define BRIGHT_THRESHOLD 150  // величина сигнала, ниже которой яркость переключится на минимум (0-1023)
#define LED_BRIGHT_MAX 255    // макс яркость светодиода СО2 (0 - 255)
#define LED_BRIGHT_MIN 2     // мин яркость светодиода СО2 (0 - 255)
#define LCD_BRIGHT_MAX 255    // макс яркость подсветки дисплея (0 - 255)
#define LCD_BRIGHT_MIN 5     // мин яркость подсветки дисплея (0 - 255)

// отключение диода в заданный промежуток времени
#define LED_TIME 2          // 0/1 - вкл/откл отключение диода по времени, 2 - отключение диода по времени, если яркость ниже LED_ON_THRESHOLD
#define LED_TIME_ON 8       // час, после которого диод загорится
#define LED_TIME_OFF 22     // час, после которого диод потухнет
#define LED_ON_THRESHOLD 30 // уровень яркости, ниже которого диод не будет отключаться

#define BLUE_YELLOW 1       // жёлтый цвет вместо синего (1 да, 0 нет) но из за особенностей подключения жёлтый не такой яркий
#define DISP_MODE 2         // в правом верхнем углу отображать: 0 - год, 1 - день недели, 2 - секунды
#define WEEK_LANG 0         // язык дня недели: 0 - английский, 1 - русский (транслит)
#define DEBUG 0             // вывод на дисплей лог инициализации датчиков при запуске. Для дисплея 1602 не работает! Но дублируется через порт!
#define PRESSURE 0          // 0 - график давления, 1 - график прогноза дождя (вместо давления). Не забудь поправить пределы гроафика
#define CO2_SENSOR 1        // включить или выключить поддержку/вывод с датчика СО2 (1 вкл, 0 выкл)
#define DISPLAY_TYPE 1      // тип дисплея: 1 - 2004 (большой), 0 - 1602 (маленький)
#define DISPLAY_ADDR 0x27   // адрес платы дисплея: 0x27 или 0x3f. Если дисплей не работает - смени адрес! На самом дисплее адрес не указан
#define BME_ADDR 0x76       // адрес сенсора BME280: 0x76 или 0x77
#define BATTERY_LEVEL 1     // 0/1 - откл./вкл. отображение заряда аккумулятора

#define VCC_CALIBRATION 0 // калибровка вольтметра

// пределы отображения для графиков
#define TEMP_MIN 15
#define TEMP_MAX 35
#define HUM_MIN 0
#define HUM_MAX 100
#define PRESS_MIN 700
#define PRESS_MAX 800
#define CO2_MIN 300
#define CO2_MAX 4000

// константа для вольтметра
float MY_VCC_CONST = 1.081;

// настройки отображений графиков
byte MAX_ONDATA = 1 + 2 + 16 + 32; // максимальные показания графиков исходя из накопленных фактических (но в пределах лимитов) данных вместо указанных пределов, 0 - использовать фиксированные пределы (с)НР
byte MIN_ONDATA = 1 + 2 + 16 + 32; // минимальные показания графиков исходя из накопленных фактических (но в пределах лимитов) данных вместо указанных пределов, 0 - использовать фиксированные пределы (с)НР
/* 1 - для графика температуры часовой, 2 - для графика температуры суточной (с)НР
   4 - для графика влажности часовой, 8 - для графика влажности суточной (с)НР
   16 - для графика давления часового, 32 - для графика давления суточного (с)НР
   64 - для графика СО2 часового, 128 - для графика СО2 суточного (с)НР
   для выборочных графиков значения нужно сложить (с)НР
   например: для изменения пределов у графиков суточной температуры и суточного СО2 складываем 2 + 128 и устанавливаем значение 130 (можно ставить сумму) (с)НР
*/

// если дисплей не заводится - поменяйте адрес (строка 54)

// пины
#define BACKLIGHT 10     // пин подсветки дисплея
#define PHOTO A3         // пин фоторезистора

#define MHZ_RX 2
#define MHZ_TX 3

#define LED_COM 7
#define LED_R 9
#define LED_G 6
#define LED_B 5

#define BTN_PIN 4

#define BATTERY A7

// библиотеки
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#if (DISPLAY_TYPE == 1)
LiquidCrystal_I2C lcd(DISPLAY_ADDR, 20, 4);
#else
LiquidCrystal_I2C lcd(DISPLAY_ADDR, 16, 2);
#endif

#include "RTClib.h"
RTC_DS3231 rtc;
DateTime now;

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

#if (CO2_SENSOR == 1)
#include <MHZ19_uart.h>
MHZ19_uart mhz19;
#endif

#include <GyverTimer.h>
GTimer_ms sensorsTimer(SENS_TIME);
GTimer_ms drawSensorsTimer(SENS_TIME);
GTimer_ms clockTimer(500);
GTimer_ms hourPlotTimer((long)4 * 60 * 1000);         // 4 минуты
GTimer_ms dayPlotTimer((long)1.6 * 60 * 60 * 1000);   // 1.6 часа
GTimer_ms plotTimer(240000);
GTimer_ms predictTimer((long)10 * 60 * 1000);         // 10 минут
GTimer_ms brightTimer(2000);
GTimer_ms co2_calibrationTimer((long)30 * 60 * 1000);

#include "GyverButton.h"
GButton button(BTN_PIN, LOW_PULL, NORM_OPEN);

// вольтметр
#if (BATTERY_LEVEL == 1)
int bat_vol, bat_old, bat_vol_f;
float filter_k = 0.04;
#endif

// датчик освещения
int light, bright, lcd_bright, led_bright;

//калибровка датчика СО2
bool co2_calibration = false;
int cal_timer;

int8_t hrs, mins, secs;
byte mode = 0;
/*
  0 часы и данные
  1 график углекислого за час
  2 график углекислого за сутки
  3 график температуры за час
  4 график температуры за сутки
  5 график влажности за час
  6 график влажности за сутки
  7 график давления за час
  8 график давления за сутки
*/

// переменные для вывода
float dispTemp;
byte dispHum;
int dispPres;
int dispCO2;
int dispRain;
int dispBat;

// массивы графиков
int tempHour[15], tempDay[15];
int humHour[15], humDay[15];
int pressHour[15], pressDay[15];
int co2Hour[15], co2Day[15];
int delta;
uint32_t pressure_array[6];
uint32_t sumX, sumY, sumX2, sumXY;
float a, b;
byte time_array[6];

// символы
// график
byte rowS[8] = {0b00000,  0b00000,  0b00000,  0b00000,  0b10001,  0b01010,  0b00100,  0b00000}; // стрелка вниз (с)НР
byte row7[8] = {0b00000,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
byte row6[8] = {0b00000,  0b00000,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
byte row5[8] = {0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
byte row4[8] = {0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111,  0b11111};
byte row3[8] = {0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111};
byte row2[8] = {0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111};
byte row1[8] = {0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111};

// цифры
uint8_t LT[8] = {0b00111,  0b01111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
uint8_t UB[8] = {0b11111,  0b11111,  0b11111,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000};
uint8_t RT[8] = {0b11100,  0b11110,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
uint8_t LL[8] = {0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b01111,  0b00111};
uint8_t LB[8] = {0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111};
uint8_t LR[8] = {0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11110,  0b11100};
uint8_t UMB[8] = {0b11111,  0b11111,  0b11111,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111};
uint8_t LMB[8] = {0b11111,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111};

void drawDig(byte dig, byte x, byte y) {
  switch (dig) {
    case 0:
      lcd.setCursor(x, y); // set cursor to column 0, line 0 (first row)
      lcd.write(0);  // call each segment to create
      lcd.write(1);  // top half of the number
      lcd.write(2);
      lcd.setCursor(x, y + 1); // set cursor to colum 0, line 1 (second row)
      lcd.write(3);  // call each segment to create
      lcd.write(4);  // bottom half of the number
      lcd.write(5);
      break;
    case 1:
      lcd.setCursor(x + 1, y);
      lcd.write(2);
      lcd.setCursor(x + 1, y + 1);
      lcd.write(3);
      break;
    case 2:
      lcd.setCursor(x, y);
      lcd.write(6);
      lcd.write(6);
      lcd.write(2);
      lcd.setCursor(x, y + 1);
      lcd.write(3);
      lcd.write(7);
      lcd.write(7);
      break;
    case 3:
      lcd.setCursor(x, y);
      lcd.write(6);
      lcd.write(6);
      lcd.write(2);
      lcd.setCursor(x, y + 1);
      lcd.write(7);
      lcd.write(7);
      lcd.write(5);
      break;
    case 4:
      lcd.setCursor(x, y);
      lcd.write(3);
      lcd.write(4);
      lcd.write(2);
      lcd.setCursor(x + 2, y + 1);
      lcd.write(5);
      break;
    case 5:
      lcd.setCursor(x, y);
      lcd.write(0);
      lcd.write(6);
      lcd.write(6);
      lcd.setCursor(x, y + 1);
      lcd.write(7);
      lcd.write(7);
      lcd.write(5);
      break;
    case 6:
      lcd.setCursor(x, y);
      lcd.write(0);
      lcd.write(6);
      lcd.write(6);
      lcd.setCursor(x, y + 1);
      lcd.write(3);
      lcd.write(7);
      lcd.write(5);
      break;
    case 7:
      lcd.setCursor(x, y);
      lcd.write(1);
      lcd.write(1);
      lcd.write(2);
      lcd.setCursor(x + 1, y + 1);
      lcd.write(0);
      break;
    case 8:
      lcd.setCursor(x, y);
      lcd.write(0);
      lcd.write(6);
      lcd.write(2);
      lcd.setCursor(x, y + 1);
      lcd.write(3);
      lcd.write(7);
      lcd.write(5);
      break;
    case 9:
      lcd.setCursor(x, y);
      lcd.write(0);
      lcd.write(6);
      lcd.write(2);
      lcd.setCursor(x, y + 1);
      lcd.write(7);
      lcd.write(7);
      lcd.write(5);
      break;
  }
}

void drawdots(byte x, byte y, boolean state) {
  byte code;
  if (state) code = 165;
  else code = 32;
  lcd.setCursor(x, y);
  lcd.write(code);
  lcd.setCursor(x, y + 1);
  lcd.write(code);
}

void drawClock(byte hours, byte minutes, byte x, byte y, boolean dotState) {
  // чисти чисти!
  lcd.setCursor(x, y);
  lcd.print("               ");
  lcd.setCursor(x, y + 1);
  lcd.print("               ");

  drawDig(hours / 10, x, y);
  drawDig(hours % 10, x + 4, y);
  // тут должны быть точки. Отдельной функцией
  drawDig(minutes / 10, x + 8, y);
  drawDig(minutes % 10, x + 12, y);
}

#if (WEEK_LANG == 0)
static const char *dayNames[]  = {
  "Sund",
  "Mond",
  "Tues",
  "Wedn",
  "Thur",
  "Frid",
  "Satu",
};
#else
static const char *dayNames[]  = {
  "BOCK",
  "POND",
  "BTOP",
  "CPED",
  "4ETB",
  "5YAT",
  "CYBB",
};
#endif

void drawData() {
  lcd.setCursor(15, 0);
  if (now.day() < 10) lcd.print(0);
  lcd.print(now.day());
  lcd.print(".");
  if (now.month() < 10) lcd.print(0);
  lcd.print(now.month());

  if (DISP_MODE == 0) {
    lcd.setCursor(16, 1);
    lcd.print(now.year());
  } else if (DISP_MODE == 1) {
    lcd.setCursor(16, 1);
    int dayofweek = now.dayOfTheWeek();
    lcd.print(dayNames[dayofweek]);
  }
}

void drawPlot(byte pos, byte row, byte width, byte height, int min_val, int max_val, int *plot_array, String label, int stretch) {
  int max_value = -32000;
  int min_value = 32000;

  for (byte i = 0; i < 15; i++) {
    if (plot_array[i] > max_value) max_value = plot_array[i];
    if (plot_array[i] < min_value) min_value = plot_array[i];
  }

  // меняем пределы графиков на предельные/фактические значения (в пределах установленных лимитов), одновременно рисуем указатель пределов (стрелочки вверх-вниз) (с)НР
  lcd.setCursor(15, 0);
  if ((MAX_ONDATA & (1 << (stretch - 1))) > 0) {    // побитовое сравнение 1 - растягиваем, 0 - не растягиваем (по указанным пределам) (с)НР
    if (max_val >= max_value) max_val = max_value;
    lcd.write(0b01011110);
  }  else {
    lcd.write(0);
  }

  lcd.setCursor(15, 3);
  if ((MIN_ONDATA & (1 << (stretch - 1))) > 0) {    // побитовое сравнение 1 - растягиваем, 0 - не растягиваем (по указанным пределам) (с)НР
    if (min_val <= min_value) min_val = min_value;
    lcd.write(0);
  } else {
    lcd.write(0b01011110);
  }
  lcd.setCursor(15, 1); lcd.write(0b01111100);
  lcd.setCursor(15, 2); lcd.write(0b01111100);

  if (min_val >= max_val) max_val = min_val + 1;

  lcd.setCursor(16, 0); lcd.print(max_value);
  lcd.setCursor(16, 1); lcd.print(label);
  lcd.setCursor(16, 2); lcd.print(plot_array[14]);
  lcd.setCursor(16, 3); lcd.print(min_value);

  for (byte i = 0; i < width; i++) {                  // каждый столбец параметров
    int fill_val = plot_array[i];
    fill_val = constrain(fill_val, min_val, max_val);
    byte infill, fract;
    // найти количество целых блоков с учётом минимума и максимума для отображения на графике
    if (plot_array[i] > min_val)
      infill = floor((float)(plot_array[i] - min_val) / (max_val - min_val) * height * 10);
    else infill = 0;
    fract = (float)(infill % 10) * 8 / 10;                   // найти количество оставшихся полосок
    infill = infill / 10;

    for (byte n = 0; n < height; n++) {     // для всех строк графика
      if (n < infill && infill > 0) {       // пока мы ниже уровня
        lcd.setCursor(i, (row - n));        // заполняем полными ячейками
        lcd.write(255);
      }
      if (n >= infill) {                    // если достигли уровня
        lcd.setCursor(i, (row - n));
        if (n == 0 && fract == 0) fract++;      // если нижний перел графика имеет минимальное значение, то рисуем одну полоску, чтобы не было пропусков (с)НР
        if (fract > 0) lcd.write(fract);        // заполняем дробные ячейки
        else lcd.write(16);                     // если дробные == 0, заливаем пустой
        for (byte k = n + 1; k < height; k++) { // всё что сверху заливаем пустыми
          lcd.setCursor(i, (row - k));
          lcd.write(16);
        }
        break;
      }
    }
  }
}

void loadClock() {
  lcd.createChar(0, LT);
  lcd.createChar(1, UB);
  lcd.createChar(2, RT);
  lcd.createChar(3, LL);
  lcd.createChar(4, LB);
  lcd.createChar(5, LR);
  lcd.createChar(6, UMB);
  lcd.createChar(7, LMB);
}

void loadPlot() {
  lcd.createChar(0, rowS); // Стрелка вниз для индикатора пределов (с)НР
  lcd.createChar(1, row1);
  lcd.createChar(2, row2);
  lcd.createChar(3, row3);
  lcd.createChar(4, row4);
  lcd.createChar(5, row5);
  lcd.createChar(6, row6);
  lcd.createChar(7, row7);
}

#if (LED_MODE == 0)
byte LED_ON = (LED_BRIGHT_MAX);
#else
byte LED_ON = (255 - LED_BRIGHT_MAX);
#endif

void setLED(byte color) {
  // сначала всё выключаем
  if (!LED_MODE) {
    analogWrite(LED_R, 0);
    analogWrite(LED_G, 0);
    analogWrite(LED_B, 0);
  } else {
    analogWrite(LED_R, 255);
    analogWrite(LED_G, 255);
    analogWrite(LED_B, 255);
  }
  if ((LED_TIME == 0) ||  (LED_TIME_ON <= hrs && LED_TIME_OFF > hrs) || (LED_TIME == 2 && LED_ON >= LED_ON_THRESHOLD)) {
    switch (color) {    // 0 выкл, 1 красный, 2 зелёный, 3 синий (или жёлтый)
      case 0:
        break;
      case 1: analogWrite(LED_R, LED_ON);
        break;
      case 2: analogWrite(LED_G, LED_ON);
        break;
      case 3:
        if (!BLUE_YELLOW) analogWrite(LED_B, LED_ON);
        else {
          //analogWrite(LED_R, LED_ON - 50);    // чутка уменьшаем красный
          //analogWrite(LED_G, LED_ON);
          analogWrite(LED_R, LED_ON);  // убираем умеьшение красного (с)НР
          analogWrite(LED_G, LED_ON / 2);       // Снижаем зеленый, чтобы был больше похож на желтый (с)НР
        }
        break;
    }
  }
}

void setup() {
  Serial.begin(9600);

#if (BATTERY_LEVEL == 1)
  if (VCC_CALIBRATION) vcc_cal();

  bat_old = analogRead(BATTERY) * readVcc() / 1023;
#endif

  co2_calibrationTimer.stop();

  pinMode(BACKLIGHT, OUTPUT);
  pinMode(LED_COM, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  digitalWrite(LED_COM, LED_MODE);
  analogWrite(BACKLIGHT, LCD_BRIGHT_MAX);

  lcd.init();
  lcd.backlight();
  lcd.clear();

#if (DEBUG == 1 && DISPLAY_TYPE == 1)
  boolean status = true;

  setLED(1);

#if (CO2_SENSOR == 1)
  lcd.setCursor(0, 0);
  lcd.print(F("MHZ-19... "));
  Serial.print(F("MHZ-19... "));
  mhz19.begin(MHZ_TX, MHZ_RX);
  mhz19.setAutoCalibration(false);
  mhz19.getStatus();    // первый запрос, в любом случае возвращает -1
  delay(500);
  if (mhz19.getStatus() == 0) {
    lcd.print(F("OK"));
    Serial.println(F("OK"));
  } else {
    lcd.print(F("ERROR"));
    Serial.println(F("ERROR"));
    status = false;
  }
#endif

  setLED(2);
  lcd.setCursor(0, 1);
  lcd.print(F("RTC... "));
  Serial.print(F("RTC... "));
  delay(50);
  if (rtc.begin()) {
    lcd.print(F("OK"));
    Serial.println(F("OK"));
  } else {
    lcd.print(F("ERROR"));
    Serial.println(F("ERROR"));
    status = false;
  }

  setLED(3);
  lcd.setCursor(0, 2);
  lcd.print(F("BME280... "));
  Serial.print(F("BME280... "));
  delay(50);
  if (bme.begin(BME_ADDR, &Wire)) {
    lcd.print(F("OK"));
    Serial.println(F("OK"));
  } else {
    lcd.print(F("ERROR"));
    Serial.println(F("ERROR"));
    status = false;
  }

  setLED(0);
  lcd.setCursor(0, 3);
  if (status) {
    lcd.print(F("All good"));
    Serial.println(F("All good"));
  } else {
    lcd.print(F("Check wires!"));
    Serial.println(F("Check wires!"));
  }
  while (1) {
    lcd.setCursor(14, 1);
    lcd.print("P:    ");
    lcd.setCursor(16, 1);
    lcd.print(analogRead(PHOTO), 1);
    Serial.println(analogRead(PHOTO));
    delay(300);
  }
#else

#if (CO2_SENSOR == 1)
  mhz19.begin(MHZ_TX, MHZ_RX);
  mhz19.setAutoCalibration(false);
#endif
  rtc.begin();
  bme.begin(BME_ADDR, &Wire);
#endif

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );

  if (RESET_CLOCK || rtc.lostPower())
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  now = rtc.now();
  secs = now.second();
  mins = now.minute();
  hrs = now.hour();

  bme.takeForcedMeasurement();
  uint32_t Pressure = bme.readPressure();
  for (byte i = 0; i < 6; i++) {   // счётчик от 0 до 5
    pressure_array[i] = Pressure;  // забить весь массив текущим давлением
    time_array[i] = i;             // забить массив времени числами 0 - 5
  }

  setLED(0);
  setLED(1);
  delay(1000);
  setLED(3);
  delay(1000);
  setLED(2);
  delay(1000);
  setLED(0);

  if (DISPLAY_TYPE == 1) {
    loadClock();
    drawClock(hrs, mins, 0, 0, 1);
    drawData();
  }
  readSensors();
  drawSensors();
}

void loop() {
#if (BRIGHT_CONTROL != 0)
  if (brightTimer.isReady()) checkBrightness(); // яркость
#endif
  if (sensorsTimer.isReady()) readSensors();    // читаем показания датчиков с периодом SENS_TIME

  if (co2_calibration) {
    if (co2_calibrationTimer.isReady()) co2_calibrate();
  }

#if (DISPLAY_TYPE == 1)
  if (clockTimer.isReady()) clockTick();        // два раза в секунду пересчитываем время и мигаем точками
  plotSensorsTick();                            // тут внутри несколько таймеров для пересчёта графиков (за час, за день и прогноз)
  modesTick();                                  // тут ловим нажатия на кнопку и переключаем режимы
  if (mode == 0) {                                  // в режиме "главного экрана"
    if (drawSensorsTimer.isReady()) drawSensors();  // обновляем показания датчиков на дисплее с периодом SENS_TIME
  } else if (mode < 9) {                                          // в любом из графиков
    if (plotTimer.isReady()) redrawPlot();          // перерисовываем график
  }
#else
  if (drawSensorsTimer.isReady()) drawSensors();
#endif
}
