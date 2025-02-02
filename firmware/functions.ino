void checkBrightness() {
  if (BRIGHT_CONTROL == 1) {
    if (analogRead(PHOTO) < BRIGHT_THRESHOLD) {   // если темно
      analogWrite(BACKLIGHT, LCD_BRIGHT_MIN);
#if (LED_MODE == 0)
      LED_ON = (LED_BRIGHT_MIN);
#else
      LED_ON = (255 - LED_BRIGHT_MIN);
#endif
    } else {                                      // если светло
      analogWrite(BACKLIGHT, LCD_BRIGHT_MAX);
#if (LED_MODE == 0)
      LED_ON = (LED_BRIGHT_MAX);
#else
      LED_ON = (255 - LED_BRIGHT_MAX);
#endif
    }
  } else {

    int averLight = 0;
    for (byte i = 0; i < 10; i++) {
      averLight += analogRead(PHOTO);
    }
    averLight /= 10;
    light = map(averLight, 0, 1023, 0, 255);

    //light = map(analogRead(PHOTO), 0, 1023, 0, 255);

    bright = round(light / 10) * 10;

    if (bright < LCD_BRIGHT_MIN) lcd_bright = LCD_BRIGHT_MIN;
    else if (bright > LCD_BRIGHT_MAX) lcd_bright = LCD_BRIGHT_MAX;
    else lcd_bright = bright;

    if (bright < LED_BRIGHT_MIN) led_bright = LED_BRIGHT_MIN;
    else if (bright > LED_BRIGHT_MAX) led_bright = LED_BRIGHT_MAX;
    else led_bright = bright;

    //Serial.print("resistor: "); Serial.println(light);
    //Serial.print("bright: "); Serial.println(bright);
    //Serial.print("LCD: "); Serial.println(lcd_bright);
    //Serial.print("LED: "); Serial.println(led_bright);

    analogWrite(BACKLIGHT, lcd_bright);

#if (LED_MODE == 0)
    LED_ON = (led_bright);
#else
    LED_ON = (255 - led_bright);
#endif
  }
  if (dispCO2 < 800) setLED(2);
  else if (dispCO2 < 1200) setLED(3);
  else if (dispCO2 < 1500) setLED(1);
}

void modesTick() {
  button.tick();
  boolean changeFlag = false;

  if (mode == 99 || mode == 98) {
    if (button.isDouble()) {
      mode = 0;
      changeFlag = true;
    }
    if (button.hasClicks()) {                 // проверка на наличие нажатий
      if (button.getClicks() == 4) {
        if (mode == 99) {
          co2_calibration = true;
          cal_timer = 30;
          co2_calibrationTimer.start();
          co2_calibrationTimer.reset();
        } else {
          co2_calibration = false;
          co2_calibrationTimer.stop();
        }
        mode = 0;
        changeFlag = true;
      }
    }
  } else {
    if (button.isSingle()) {
      mode++;
      if (mode > 8) mode = 0;
#if (CO2_SENSOR == 0 && mode == 1)
      mode = 3;
#endif
      changeFlag = true;
    }
    if (button.isDouble()) {                  // двойное нажатие (с)НР ----------------------------
      if (mode > 0) {                         // Меняет пределы графика на установленные/фактические максимумы (с)НР
        int bt = 1 << (mode - 1);
        if ((MAX_ONDATA & bt) > 0) {
          MAX_ONDATA = MAX_ONDATA - bt;
        }
        else {
          MAX_ONDATA = MAX_ONDATA + bt;
        }
        if ((MIN_ONDATA & bt) > 0) {
          MIN_ONDATA = MIN_ONDATA - bt;
        }
        else {
          MIN_ONDATA = MIN_ONDATA + bt;
        }
      }
      changeFlag = true;
    }
    if (button.hasClicks()) {                 // проверка на наличие нажатий
      if (button.getClicks() == 4 && mode == 2) {
        if (!co2_calibration) {
          mode = 99;
        } else {
          mode = 98;
        }
        changeFlag = true;
      }
    }
    if (button.isHolded()) {
      mode = 0;
      changeFlag = true;
    }
  }
  if (changeFlag) {
    if (mode == 0) {
      lcd.clear();
      loadClock();
      drawClock(hrs, mins, 0, 0, 1);
      if (DISPLAY_TYPE == 1) drawData();
      drawSensors();
      if (co2_calibration) {
        lcd.setCursor(15, 1);
        lcd.print("cal" + String(cal_timer));
      }
    } else if (mode == 99 || mode == 98) {
      lcd.clear();
      if (mode == 99) {
        lcd.setCursor(4, 0);
        lcd.print("Proceed with");
      } else {
        lcd.setCursor(7, 0);
        lcd.print("Cancel");
      }
      lcd.setCursor(2, 1);
      lcd.print("CO2 calibration?");
      lcd.setCursor(2, 3);
      lcd.print("(4)Yes     (2)No");
    } else {
      lcd.clear();
      loadPlot();
      redrawPlot();
    }
  }
}

void redrawPlot() {
  lcd.clear();
  switch (mode) {
    case 1: drawPlot(0, 3, 15, 4, CO2_MIN, CO2_MAX, (int*)co2Hour, "c hr", mode);
      break;
    case 2: drawPlot(0, 3, 15, 4, CO2_MIN, CO2_MAX, (int*)co2Day, "c day", mode);
      break;
    case 3: drawPlot(0, 3, 15, 4, TEMP_MIN, TEMP_MAX, (int*)tempHour, "t hr", mode);
      break;
    case 4: drawPlot(0, 3, 15, 4, TEMP_MIN, TEMP_MAX, (int*)tempDay, "t day", mode);
      break;
    case 5: drawPlot(0, 3, 15, 4, HUM_MIN, HUM_MAX, (int*)humHour, "h hr", mode);
      break;
    case 6: drawPlot(0, 3, 15, 4, HUM_MIN, HUM_MAX, (int*)humDay, "h day", mode);
      break;
    case 7: drawPlot(0, 3, 15, 4, PRESS_MIN, PRESS_MAX, (int*)pressHour, "p hr", mode);
      break;
    case 8: drawPlot(0, 3, 15, 4, PRESS_MIN, PRESS_MAX, (int*)pressDay, "p day", mode);
      break;
  }
}

void readSensors() {
  bme.takeForcedMeasurement();
  dispTemp = bme.readTemperature();
  dispHum = bme.readHumidity();
  dispPres = (float)bme.readPressure() * 0.00750062;

#if (BATTERY_LEVEL == 1)
  for (byte i = 0; i < 10; i++) {
    analogRead(BATTERY);   // отсев первых 10 измерений
  }
  int averVoltage = 0;
  for (byte i = 0; i < 10; i++) {
    averVoltage += analogRead(BATTERY);
  }
  averVoltage /= 10;
  bat_vol = (float)averVoltage * readVcc() / 1023;

  //bat_vol = analogRead(BATTERY) * readVcc() / 1023;
  bat_vol_f = filter_k * bat_vol + (1 - filter_k) * bat_old;
  bat_old = bat_vol_f;
  dispBat = map(bat_vol_f, 3400, 4200, 0, 99);

  if (dispBat > 99) {
    dispBat = 99;
  }

  //Serial.print("bat_vol is: "); Serial.println(bat_vol);
  //Serial.print("VCC is: "); Serial.println(dispBat);
  //Serial.print("VCC % is: "); Serial.println(map(dispBat, 3600, 4200, 0, 100));
#endif

#if (CO2_SENSOR == 1)
  dispCO2 = mhz19.getPPM();

  if (dispCO2 < 800) setLED(2);
  else if (dispCO2 < 1200) setLED(3);
  else if (dispCO2 >= 1200) setLED(1);
#endif
}

void drawSensors() {
#if (DISPLAY_TYPE == 1)
  // дисплей 2004
  lcd.setCursor(0, 2);
  lcd.print(String(dispTemp, 1));
  lcd.write(223);
  lcd.setCursor(6, 2);
  lcd.print(" " + String(dispHum) + "%  ");

#if (CO2_SENSOR == 1)
  lcd.print(String(dispCO2) + " ppm");
  if (dispCO2 < 1000) lcd.print(" ");
#endif

#if (BATTERY_LEVEL == 1)
  lcd.setCursor(0, 3);
  lcd.print(String(dispPres) + "mm  rain ");
  lcd.print(F("       "));
  lcd.setCursor(12, 3);
  lcd.print(String(dispRain) + "%");
  lcd.setCursor(17, 3);
  lcd.print(String(dispBat) + "%");
#else
  lcd.setCursor(0, 3);
  lcd.print(String(dispPres) + " mm  rain ");
  lcd.print(F("       "));
  lcd.setCursor(13, 3);
  lcd.print(String(dispRain) + "%");
#endif

#else
  // дисплей 1602
  lcd.setCursor(0, 0);
  lcd.print(String(dispTemp, 1));
  lcd.write(223);
  lcd.setCursor(6, 0);
  lcd.print(String(dispHum) + "% ");

#if (CO2_SENSOR == 1)
  lcd.print(String(dispCO2) + "ppm");
  if (dispCO2 < 1000) lcd.print(" ");
#endif

  lcd.setCursor(0, 1);
  lcd.print(String(dispPres) + " mm  rain ");
  lcd.print(String(dispRain) + "% ");
#endif
}

void plotSensorsTick() {
  // 4 минутный таймер
  if (hourPlotTimer.isReady()) {
    for (byte i = 0; i < 14; i++) {
      tempHour[i] = tempHour[i + 1];
      humHour[i] = humHour[i + 1];
      pressHour[i] = pressHour[i + 1];
      co2Hour[i] = co2Hour[i + 1];
    }
    tempHour[14] = dispTemp;
    humHour[14] = dispHum;
    co2Hour[14] = dispCO2;

    if (PRESSURE) pressHour[14] = dispRain;
    else pressHour[14] = dispPres;
  }

  // 1.5 часовой таймер
  if (dayPlotTimer.isReady()) {
    long averTemp = 0, averHum = 0, averPress = 0, averCO2 = 0;

    for (byte i = 0; i < 15; i++) {
      averTemp += tempHour[i];
      averHum += humHour[i];
      averPress += pressHour[i];
      averCO2 += co2Hour[i];
    }
    averTemp /= 15;
    averHum /= 15;
    averPress /= 15;
    averCO2 /= 15;

    for (byte i = 0; i < 14; i++) {
      tempDay[i] = tempDay[i + 1];
      humDay[i] = humDay[i + 1];
      pressDay[i] = pressDay[i + 1];
      co2Day[i] = co2Day[i + 1];
    }
    tempDay[14] = averTemp;
    humDay[14] = averHum;
    pressDay[14] = averPress;
    co2Day[14] = averCO2;
  }

  // 10 минутный таймер
  if (predictTimer.isReady()) {
    // тут делаем линейную аппроксимацию для предсказания погоды
    long averPress = 0;
    for (byte i = 0; i < 10; i++) {
      bme.takeForcedMeasurement();
      averPress += bme.readPressure();
      delay(1);
    }
    averPress /= 10;

    for (byte i = 0; i < 5; i++) {                   // счётчик от 0 до 5 (да, до 5. Так как 4 меньше 5)
      pressure_array[i] = pressure_array[i + 1];     // сдвинуть массив давлений КРОМЕ ПОСЛЕДНЕЙ ЯЧЕЙКИ на шаг назад
    }
    pressure_array[5] = averPress;                    // последний элемент массива теперь - новое давление
    sumX = 0;
    sumY = 0;
    sumX2 = 0;
    sumXY = 0;
    for (int i = 0; i < 6; i++) {                    // для всех элементов массива
      sumX += time_array[i];
      sumY += (long)pressure_array[i];
      sumX2 += time_array[i] * time_array[i];
      sumXY += (long)time_array[i] * pressure_array[i];
    }
    a = 0;
    a = (long)6 * sumXY;             // расчёт коэффициента наклона приямой
    a = a - (long)sumX * sumY;
    a = (float)a / (6 * sumX2 - sumX * sumX);
    delta = a * 6;      // расчёт изменения давления
    dispRain = map(delta, -250, 250, 100, -100);  // пересчитать в проценты
    //Serial.println(String(pressure_array[5]) + " " + String(delta) + " " + String(dispRain));   // дебаг
  }
}

boolean dotFlag;
void clockTick() {
  dotFlag = !dotFlag;
  if (dotFlag) {          // каждую секунду пересчёт времени
    secs++;
    if (secs > 59) {      // каждую минуту
      secs = 0;
      mins++;
      cal_timer--;
      if (mins <= 59 && mode == 0) drawClock(hrs, mins, 0, 0, 1);
    }
    if (mins > 59) {      // каждый час
      now = rtc.now();
      secs = now.second();
      mins = now.minute();
      hrs = now.hour();
      if (mode == 0) drawClock(hrs, mins, 0, 0, 1);
      if (hrs > 23) {
        hrs = 0;
      }
      if (mode == 0 && DISPLAY_TYPE) drawData();
    }
    if (DISP_MODE == 2 && mode == 0) {
      if (co2_calibration) {
        lcd.setCursor(15, 1);
        if (cal_timer < 10 && cal_timer >= 0) {
          lcd.print("cal " + String(cal_timer));
        } else {
          lcd.print("cal" + String(cal_timer));
        }
      } else {
        lcd.setCursor(16, 1);
        if (secs < 10) lcd.print("0");
        lcd.print(secs);
      }
    }
  }
  if (mode == 0) drawdots(7, 0, dotFlag);
  if (dispCO2 >= 1500) {
    if (dotFlag) setLED(1);
    else setLED(0);
  }
}

#if (BATTERY_LEVEL == 1)
void vcc_cal() {
  //--------калибровка----------
  MY_VCC_CONST = 1.1;
  Serial.print("Real VCC is: "); Serial.println(readVcc());     // общаемся с пользователем
  Serial.println("Write your VCC (in millivolts)");
  while (Serial.available() == 0); int Vcc = Serial.parseInt(); // напряжение от пользователя
  float real_const = (float)1.1 * Vcc / readVcc();              // расчёт константы
  Serial.print("New voltage constant: "); Serial.println(real_const, 3);
  while (1); // уйти в бесконечный цикл
  //------конец калибровки-------
}

long readVcc() { //функция чтения внутреннего опорного напряжения, универсальная (для всех ардуин)
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low;

  result = MY_VCC_CONST * 1023 * 1000 / result; // расчёт реального VCC
  return result; // возвращает VCC
}
#endif

void co2_calibrate() {
  mode = 95;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1st calibration now");
  mhz19.calibrateZero();
  lcd.setCursor(0, 1);
  lcd.print("wait 60sec");
  delay(60000);
  lcd.setCursor(0, 2);
  lcd.print("2nd calibration now");
  mhz19.calibrateZero();  // Just in case
  co2_calibration = false;
  co2_calibrationTimer.stop();
  delay(500);
  mhz19.getPPM();
  delay(500);
  lcd.setCursor(0, 3);
  lcd.print("Bef:" + String(dispCO2) + " Aft:" + String(mhz19.getPPM()));
}
