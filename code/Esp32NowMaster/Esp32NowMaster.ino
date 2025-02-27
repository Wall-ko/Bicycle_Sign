#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

// Определяем задействованные пины
// Для LED
#define LED_GLIM 25
#define LED_LEFT 26
#define LED_RIGHT 27
#define LED_COUNT 5
//Для кнопок
#define KEY_GLIM 34
#define KEY_LEFT 35
#define KEY_RIGHT 33

//Устанавливаем углы, ускорение и крен срабатывания поворотника
#define ANGLE1 15         //I режим
#define ANGLE2 10         //II режим
#define BOOST2 500        //II режим
#define ANGLE3 7         //III режим
#define BOOST3 300        //III режим
#define ROOL3 5           //III режим
#define BOOST_STOP 1800   //СТОП режим
#define ROOL_STOP 50       //СТОП режим

//Время работы сигнала после сброса
#define TIME1 50         //I режим
#define TIME2 30         //II режим
#define TIME3 20         //III режим
#define TIME_STOP 50     //СТОП режим
#define TIME_DIGIT 100   //РУЧНОЙ режим

// Угол сброса признака поворота
#define ANGLE_FIN 5         //


//Переменная для задержки времени опроса
#define PERIOD 100
static uint32_t tmr = 0;

//Переменная для моргания LED
#define PERIODLED 100
static uint32_t tmrLED = 0; //Переменная времи моргания LED

// Инициализируем гироскоп MPU6050
////////////////////////////////////////////////////////////////////////////////////////
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 mpu(0x69); // <-- use for AD0 high
MPU6050 mpu;

// MPU6050 переменные
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
int accX = 0;
int accY = 0;


float gyroZFix = 0;

float gyroZOld = 0;
float gyroZOld1 = 0;
float gyroZOld2 = 0;

int accXOld1 = 0;
int accXOld2 = 0;

int accYOld1 = 0;
int accYOld2 = 0;

bool FlagFix = false;

float Delta3Z = 0;

// Обьявляем переменные для передачи на другую плату с помощью протокола ESP-NOW
//////////////////////////////////////////////////////////////////////////////////////////////////
// МАС-АДРЕС ПЛАТЫ-ПОЛУЧАТЕЛЯ
//uint8_t broadcastAddress[] = {0x10, 0x06, 0x1C, 0xA6, 0xE4, 0x10};
uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x5E, 0x32, 0x6E};
//uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x5E, 0x32, 0x88};

// Структура в скетче платы-отправителя  должна совпадать с оной для получателя
typedef struct StructMessage {
  char a[1];
  int b;
} StructMessage;

// Создаем структуру сообщения myData
StructMessage myData;

// Peer info
esp_now_peer_info_t peerInfo;

////////////////////////////////////////////////////////////////////////////////////////////////////
//Объявите наш объект полосы NeoPixel:
Adafruit_NeoPixel StripGlim(LED_COUNT, LED_GLIM, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel StripLeft(LED_COUNT, LED_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel StripRight(LED_COUNT, LED_RIGHT, NEO_GRB + NEO_KHZ800);

char RouteFix;
int TimeFix = 0;
int NumLED = 0;
bool blinkFix = true;

//Переменная фиксации включения ходовых огней
bool GlimFix = false;
bool KeyFix = false;

// ================================================================
// ===                      OnDataSent                          ===
// ================================================================
// Обратная функция отправки
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nСтатус Отправки Последнего Пакета: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Сообщение доставлено" : "Сообщение НЕ доставлено");
}

// ================================================================
// ===                      INITIAL MPU6050                     ===
// ================================================================
// Функция инициализации MPU6050
void InitialMPU () {

  // initialize MPU6050
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  //начальные значения MPU6050
  mpu.setXAccelOffset(-1132);
  mpu.setYAccelOffset(-278);
  mpu.setZAccelOffset(1062);
  mpu.setXGyroOffset(132);
  mpu.setYGyroOffset(-12);
  mpu.setZGyroOffset(40);
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Wire.begin();
  Serial.begin(115200);

  //Открываем пины на чтение
  pinMode(KEY_GLIM, INPUT);
  pinMode(KEY_LEFT, INPUT);
  pinMode(KEY_RIGHT, INPUT);

  // initialize MPU6050
  InitialMPU ();

  // Выбираем режим WiFi
  WiFi.mode(WIFI_STA);

  // Запускаем протокол ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Ошибка инициализации ESP-NOW");
    return;
  }

  // Регистрируем отправку сообщения
  esp_now_register_send_cb(OnDataSent);

  // Указываем получателя
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  //ИНИЦИАЛИЗАЦИЯ объекта полосы NeoPixel
  StripGlim.begin();
  StripGlim.show();
  StripGlim.setBrightness(150);

  StripLeft.begin();
  StripLeft.show();
  StripLeft.setBrightness(150);

  StripRight.begin();
  StripRight.show();
  StripRight.setBrightness(150);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  /////////////////////////////////////////////////////////
  // костыль для обнуления датчика чтобы не привысить угол 180
  if ((gyroZ > 120) || (gyroZ < -120)) { // костыль для обнуления датчика чтобы не привысить угол 180
    gyroX = 0;
    gyroY = 0;
    gyroZ = 0;
    accX = 0;
    accY = 0;
    gyroZFix = 0;
    gyroZOld = 0;
    gyroZOld1 = 0;
    gyroZOld2 = 0;
    accXOld1 = 0;
    accXOld2 = 0;
    accYOld1 = 0;
    accYOld2 = 0;
    FlagFix = false;
    Delta3Z = 0;

    InitialMPU ();
  }

  //////////////////////////////////////////////////////////////
  //Получение и обработка данных с датчика MPU6050
  if (millis() - tmr >= PERIOD) {  // таймер получения и передачи данных
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Получить последний пакет

      //Получаем данные с MPU6050
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

      gyroZ = (ypr[0] * 180 / M_PI);
      gyroX = (ypr[1] * 180 / M_PI);
      gyroY = (ypr[2] * 180 / M_PI);
      accX = (aaReal.x + accXOld1 + accXOld2) / 3;
      accY = (aaReal.y + accYOld1 + accYOld2) / 3;

      Delta3Z = abs(gyroZOld2 - gyroZ);

      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[2] * 180 / M_PI);
      Serial.print("\t");

      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.print(aaReal.z);
      Serial.print("\t");
    }

    ///////////////////////////////////////////////////////
    //Обработка нажатия кнопок
    if (digitalRead(KEY_GLIM) && !KeyFix ) {
      strcpy(myData.a, "G");
      myData.b = 5;
      Serial.println("press Glim ON");
    }
    else if (digitalRead(KEY_GLIM) && KeyFix ) {
      strcpy(myData.a, "F");
      myData.b = 5;
      Serial.println("press Glim OFF");
    }
    else if (digitalRead(KEY_RIGHT)) {
      strcpy(myData.a, "R");
      myData.b = TIME_DIGIT;
      Serial.println("press Right");
    }
    else if (digitalRead(KEY_LEFT)) {
      strcpy(myData.a, "L");
      myData.b = TIME_DIGIT;
      Serial.println("press Left");
    }
    ///////////////////////////////////////////////////////
    //Определяем напровление поворота // направо - плюc, налево - минус
    else if ((gyroZ - gyroZFix) > ANGLE1) // по углу поворачиваем направо
    { FlagFix = true;
      strcpy(myData.a, "R");
      myData.b = TIME1;
      Serial.println("Right Gyro");
    }
    else if ((gyroZ - gyroZFix) < -ANGLE1) // по углу поворачиваем налево
    { FlagFix = true;
      strcpy(myData.a, "L");
      myData.b = TIME1;
      Serial.println("Left Gyro");
    }
    ///////////////////////////////////////////////////////
    else if (((gyroZ - gyroZFix) > ANGLE2) && (accY < -BOOST2))// по углу и ускорению поворачиваем направо
    { FlagFix = true;
      strcpy(myData.a, "R");
      myData.b = TIME2;
      Serial.println("Right Gyro&Acc");
    }
    else if (((gyroZ - gyroZFix) < -ANGLE2) && (accY > BOOST2)) // по углу и ускорению поворачиваем налево
    { FlagFix = true;
      strcpy(myData.a, "L");
      myData.b = TIME2;
      Serial.println("Left Gyro&Acc");
    }
    //////////////////////////////////////////////////////////
    else if (((gyroZ - gyroZFix) > ANGLE3) && (accY < -BOOST3) && (gyroY  > ROOL3))// по углу, ускорению и наклону поворачиваем направо
    { FlagFix = true;
      strcpy(myData.a, "R");
      myData.b = TIME3;
      Serial.println("Right Gyro&Acc&Gyro");
    }
    else if (((gyroZ - gyroZFix) < -ANGLE3) && (accY > BOOST3) && (gyroY  < -ROOL3)) // по углу, ускорению и наклону поворачиваем налево
    { FlagFix = true;
      strcpy(myData.a, "L");
      myData.b = TIME3;
      Serial.println("Left Gyro&Acc&Gyro");
    }
    //////////////////////////////////////////////////////////
    else if ((accX < -BOOST_STOP) || (gyroX < -ROOL_STOP))  // тормозим
    {
      strcpy(myData.a, "S");
      myData.b = TIME_STOP;
      Serial.println("Stop");
    }
    //////////////////////////////////////////////////////////
    else // Едим прямо
    {
      strcpy(myData.a, "N");
      myData.b = 0;
      Serial.println("NULL");
    }
    ///////////////////////////////////////////////////////
    // Отправляем сообщение
       esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    ///////////////////////////////////////////////////////////
    //Сброс признака поворота
    //Фиксируем значения для сравнения
    if (!FlagFix || (FlagFix && (Delta3Z < ANGLE_FIN))) {
      gyroZFix = gyroZ;
      FlagFix = false;
    }

    /////////////////////////////////////////////////////////////
    //Запоминием предидущий угол и ускорения
    gyroZOld2 = gyroZOld1;
    gyroZOld1 = gyroZOld;
    gyroZOld = gyroZ;

    accXOld2 = accXOld1;
    accXOld1 = aaReal.x;

    accYOld2 = accYOld1;
    accYOld1 = aaReal.y ;

    //Раздел управления LED
    ///////////////////////////////////////////////////////////////////////
    if ((myData.a[0] != RouteFix) && (myData.a[0] != 'N')) {
      RouteFix = myData.a[0];
      TimeFix =  myData.b;
      tmrLED = 0;
      NumLED = 2;
    }
    else {
      TimeFix =  TimeFix - 1;
    }

    tmr = millis();  // сброс таймера
  } // конец таймера получения данных

  //Раздел Включения LED
  ///////////////////////////////////////////////////////////////////////
  // Моргаем правым поворотом
  if ((RouteFix == 'R') && (TimeFix > 0)) {
    //Serial.println("LED Right ON");
    // действие по таймеру
    // Бегущая точка LED
    if (millis() - tmrLED >= PERIODLED) {
      tmrLED = millis();
      NumLED = (NumLED >= 4) ? 0 : NumLED + 1;
      for (int i = 0; i <= 4; i++) {
        StripRight.setPixelColor(i, ((i == NumLED) ? StripRight.Color(255, 100, 0) : StripRight.Color(0, 0, 0)));
        StripGlim.setPixelColor((i == 4) ? 0 : i + 1, ((i == NumLED) ? StripGlim.Color(255, 100, 0) : StripGlim.Color(0, 0, 0)));
        StripLeft.setPixelColor(i, ((i == NumLED) ? StripLeft.Color(255, 100, 0) : StripLeft.Color(0, 0, 0)));
      }
      StripRight.show();
      StripLeft.show();
      StripGlim.show();
    }
  }
  /////////////////////////////////////////////////////////////////
  // Моргаем левым поворотом
  else if ((RouteFix == 'L') && (TimeFix > 0)) {
    //Serial.println("LED Left ON");
    // действие по таймеру
    // Бегущая точка LED
    if (millis() - tmrLED >= PERIODLED) {
      tmrLED = millis();
      NumLED = (NumLED <= 0) ? 4 : NumLED - 1;
      for (int i = 0; i <= 4; i++) {
        StripRight.setPixelColor(i, ((i == NumLED) ? StripRight.Color(255, 100, 0) : StripRight.Color(0, 0, 0)));
        StripGlim.setPixelColor((i == 0) ? 4 : i - 1, ((i == NumLED) ? StripGlim.Color(255, 100, 0) : StripGlim.Color(0, 0, 0)));
        StripLeft.setPixelColor(i, ((i == NumLED) ? StripLeft.Color(255, 100, 0) : StripLeft.Color(0, 0, 0)));
      }
      StripRight.show();
      StripLeft.show();
      StripGlim.show();
    }
  }
  /////////////////////////////////////////////////////////////////
  // Выключаем ВСЁ
  else if ((RouteFix == 'F') || ((RouteFix == 'S') && (!GlimFix)) || ((!GlimFix) && (TimeFix < 1))) {
    //Serial.println("LED All OFF");
    KeyFix = false;
    GlimFix = false;
    RouteFix = 'N';
    TimeFix = 0;
    for (int i = 0; i <= 4; i++) {
      StripRight.setPixelColor(i, StripRight.Color(0, 0, 0));
      StripLeft.setPixelColor(i, StripLeft.Color(0, 0, 0));
      StripGlim.setPixelColor(i, StripGlim.Color(0, 0, 0));
    }
    StripRight.show();
    StripLeft.show();
    StripGlim.show();
  }
  /////////////////////////////////////////////////////////////////
  // Включаем ходовые огни
  else if ((RouteFix == 'G') || (GlimFix)) {
    if (TimeFix < 1) KeyFix = true;
    //Serial.println("LED Glim ON");
    GlimFix = true;
    for (int i = 0; i <= 4; i++) {
      StripRight.setPixelColor(i, StripRight.Color(255, 255, 255));
      StripLeft.setPixelColor(i, StripLeft.Color(255, 255, 255));
      StripGlim.setPixelColor(i, StripGlim.Color(255, 255, 255));
    }
    StripRight.show();
    StripLeft.show();
    StripGlim.show();
  }
}
