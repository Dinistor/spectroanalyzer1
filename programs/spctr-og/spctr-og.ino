//original code made by AlexGyver
//OG репозиторий тут: https://github.com/AlexGyver/FHTSpectrumAnalyzer

//я чуток твикнул код под себя выкинув лишнее
//шить через любой ISP программатор, настройки стандартные - 16 мегагерц, контроллер атмега328р. 
//Использовать можно любое ядро под атмегу, хоть MiniCore (328/168/88/48/8), хоть GyverCore (только под 328)

//работает по идее на любой атмеге 8/48/88/168/328, но я проверял только на 8 и 328. На 8 атмегу влезает впритык, поэтому её и рекомендую брать.
//вывод проги на 8 атмеге:
//Sketch uses 6546 bytes (85%) of program storage space. Maximum is 7680 bytes.
//Global variables use 950 bytes (92%) of dynamic memory, leaving 74 bytes for local variables. Maximum is 1024 bytes.
//поэтому рекомендую 8 атмегу, чтобы "раскрыть потенциал мк на полную"

//плата на 328 атмеге жрет 42ма от 12в, на 8 атмеге 53ма.

///////////  ВАЖНО!!! у 8 атмеги встроенный референс на 2,56в, поэтому требуется отдельная настройка.
//для атмеги 328 используется резистор 47к последовательно с подстроечником на 10к.
//для атмеги 8 используется резистор 10к последовательно с подстроечником на 10к.

// ---------------- НАСТРОЙКИ ----------------
//#define DRIVER_VERSION 1   // 0 - маркировка драйвера кончается на 4АТ, 1 - на 4Т
#define GAIN_CONTROL 1     // ручная настройка потенциометром на громкость (1 - вкл, 0 - выкл)

#define DEF_GAIN 80 

#define VOL_THR 35        // порог тишины (ниже него отображения на матрице не будет)

#define LOW_PASS 25        // нижний порог чувствительности шумов (нет скачков при отсутствии звука)

#define FHT_N 256          // ширина спектра х2
// вручную забитый массив тонов, сначала плавно, потом круче
byte posOffset[16] = {2, 3, 4, 6, 8, 10, 12, 14, 16, 20, 25, 30, 35, 60, 80, 100};//в принципе можно не трогать.
//первая полоска +/- на 100 герцах, ниже смысла брать нет, нулевая и первая полоски  всегда стоят колом на 100%
//если хочется чекнуть что как, смотри прогу "spektrumFHT" в оргинальной репе
//максимальное значение по Y 150

// ---------------- НАСТРОЙКИ ----------------

// ---------------------- ПИНЫ ----------------------
#define AUDIO_IN 0          // пин, куда подключен звук
#define POT_PIN 2          // пин потенциометра настройки
// ---------------------- ПИНЫ ----------------------

// --------------- БИБЛИОТЕКИ ---------------
#define LOG_OUT 1
#include <FHT.h>          // преобразование Хартли
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define printByte(args) write(args);
double prevVolts = 100.0;
// --------------- БИБЛИОТЕКИ ---------------
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


LiquidCrystal_I2C lcd(0x27, 16, 2);

// ------------------------------------- ПОЛОСОЧКИ -------------------------------------
byte v1[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111};
byte v2[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111};
byte v3[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111};
byte v4[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111};
byte v5[8] = {0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
byte v6[8] = {0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
byte v7[8] = {0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
byte v8[8] = {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
// ------------------------------------- ПОЛОСОЧКИ -------------------------------------
byte gain = DEF_GAIN;   // усиление по умолчанию
unsigned long gainTimer;
byte maxValue, maxValue_f;
float k = 0.1;

void setup() {
  // поднимаем частоту опроса аналогового порта до 38.4 кГц, по теореме
  // Котельникова (Найквиста) частота дискретизации будет 19 кГц
  // http://yaab-arduino.blogspot.ru/2015/02/fast-sampling-from-analog-input.html
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

//опора встроенная 1,1В для атмеги 328 и 2,56в для атмеги 8
  analogReference(INTERNAL);

  Wire.setClock(800000);//скорость i2c 800кГц, рудимент с дебага проблемы скачков уровня
  Wire.begin(); 

  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  lcd.createChar(0, v1);
  lcd.createChar(1, v2);
  lcd.createChar(2, v3);
  lcd.createChar(3, v4);
  lcd.createChar(4, v5);
  lcd.createChar(5, v6);
  lcd.createChar(6, v7);
  lcd.createChar(7, v8);
}

void loop() {
  gain = map(analogRead(A2), 0, 1023, 0, 150);

  analyzeAudio();   // функция FHT, забивает массив fht_log_out[] величинами по спектру

  for (int pos = 0; pos < 16; pos++) {   // для окошек дисплея с 0 по 15
    // найти максимум из пачки тонов
    if (fht_log_out[posOffset[pos]] > maxValue) maxValue = fht_log_out[posOffset[pos]];

    lcd.setCursor(pos, 0);

    // преобразовать значение величины спектра в диапазон 0..15 с учётом настроек
    int posLevel = map(fht_log_out[posOffset[pos]], LOW_PASS, gain, 0, 15);
    posLevel = constrain(posLevel, 0, 15);

    if (posLevel > 7) {               // если значение больше 7 (значит нижний квадратик будет полный)
      lcd.printByte(posLevel - 8);    // верхний квадратик залить тем что осталось
      lcd.setCursor(pos, 1);          // перейти на нижний квадратик
      lcd.printByte(7);               // залить его полностью
    } else {                          // если значение меньше 8
      lcd.print(" ");                 // верхний квадратик пустой
      lcd.setCursor(pos, 1);          // нижний квадратик
      lcd.printByte(posLevel);        // залить полосками
    }
  }

}

void analyzeAudio() {
  for (int i = 0 ; i < FHT_N ; i++) {
    int sample = analogRead(AUDIO_IN);
    fht_input[i] = sample; // put real data into bins
  }
  fht_window();  // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run();     // process the data in the fht
  fht_mag_log(); // take the output of the fht
}