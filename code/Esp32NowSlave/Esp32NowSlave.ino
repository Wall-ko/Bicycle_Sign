#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

#define LED_BACK 16
#define LED_LEFT 17
#define LED_RIGHT 18
#define LED_GLIM 21
#define LED_COUNT_BACK 28
#define LED_COUNT 14

// Структура должна совпадать со структурой
// на плате-отправителе
typedef struct StructMessage {
  char a[1];
  int b;
} StructMessage;
// Создаем myData
StructMessage myData;
boolean ReadFix = false;

//Объявите наш объект полосы NeoPixel:
Adafruit_NeoPixel StripBack(LED_COUNT_BACK, LED_BACK, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel StripLeft(LED_COUNT, LED_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel StripRight(LED_COUNT, LED_RIGHT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel StripGlim(LED_COUNT, LED_GLIM, NEO_GRB + NEO_KHZ800);

char RouteFix;
int TimeFix = 0;
boolean blinkFix = true;
uint32_t tmrOn = 0;
uint32_t tmrOff = 0;


// ================================================================
// ===                      OnDataSent                          ===
// ================================================================
// Обратная функция при получении
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  ReadFix = true;
  //  Serial.print("Bytes received: ");
  //  Serial.print(len);
  //  Serial.print("\t");
  Serial.print("Char: ");
  Serial.print(myData.a);
  Serial.print("\t");
  Serial.print("Int: ");
  Serial.print(myData.b);
  Serial.println("\t");
  //  Serial.println("");
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  // Запускаем монитор порта
  Serial.begin(115200);

  // Выставляем режим работы WiFi
  WiFi.mode(WIFI_STA);
  // Запускаем протокол ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Получаем состояние отправки
  esp_now_register_recv_cb(OnDataRecv);


  //ИНИЦИАЛИЗАЦИЯ объекта полосы NeoPixel
  StripBack.begin();
  StripBack.show();
  StripBack.setBrightness(150);

  StripLeft.begin();
  StripLeft.show();
  StripLeft.setBrightness(150);

  StripRight.begin();
  StripRight.show();
  StripRight.setBrightness(150);

  StripGlim.begin();
  StripGlim.show();
  StripGlim.setBrightness(150);

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  // фиксируем первое поступившее значение времени до отключения
  // и направление движения до его изменения
  if (ReadFix) {
    ReadFix = false;
    if ((myData.a[0] != RouteFix) && (myData.a[0] != 'N')) {
      RouteFix = myData.a[0];
      TimeFix =  myData.b;
      blinkFix = true;
      tmrOn = 0;
      tmrOff = 0;
      Serial.print("\t");
      Serial.print("RouteFix ");
      Serial.print(RouteFix);
      Serial.print("\t");
      Serial.print("TimeFix ");
      Serial.println(TimeFix);
      Serial.println("");
    }
    else {
      TimeFix =  TimeFix - 1;
    }
  }
  //    Serial.print("\t");
  //    Serial.print("RouteFix ");
  //    Serial.print(RouteFix);
  //    Serial.print("\t");
  //    Serial.print("TimeFix ");
  //    Serial.print(TimeFix);
  //    Serial.println("");


  /////////////////////////////////////////////////////////////////
  // Моргаем правым поворотом
  if ((RouteFix == 'R') && (TimeFix > 0)) {

    //    Serial.print("\t");
    //     Serial.print("blinkFix ");
    //     Serial.print(blinkFix);
    //     Serial.print("\t");
    //     Serial.print("tmrOn ");
    //     Serial.print(tmrOn);
    //     Serial.print("\t");
    //     Serial.print("millis() ");
    //     Serial.println(millis());
    //     Serial.println("");

    // действие по таймеру
    // Включаем LED на ... секунд
    if (blinkFix && (millis() >= tmrOff)) {
      tmrOn = millis() + 800;
      blinkFix = !blinkFix;
      //           правый наплечный            левый наплечный             правая стрелка                левая стрелка
      LedShow(StripRight.Color(255, 100, 0), StripLeft.Color(0, 0, 0), StripBack.Color(255, 100, 0), StripBack.Color(0, 0, 0));
      Serial.println("LED Right ON");
    }
    // Выключаем LED на ... секунд
    else if (!blinkFix  && (millis() >= tmrOn)) {
      tmrOff = millis() + 200;
      blinkFix = !blinkFix;
      //          правый наплечный          левый наплечный           правая стрелка              левая стрелка
      LedShow(StripRight.Color(0, 0, 0), StripLeft.Color(0, 0, 0), StripBack.Color(0, 0, 0), StripBack.Color(0, 0, 0));
      Serial.println("LED Right OFF");
    }
  }
  /////////////////////////////////////////////////////////////////
  // Моргаем левым поворотом
  else if ((RouteFix == 'L') && (TimeFix > 0)) {
    // действие по таймеру
    // Включаем LED на ... секунд
    if (blinkFix && (millis() >= tmrOff)) {
      tmrOn = millis() + 800;
      blinkFix = !blinkFix;
      //           правый наплечный            левый наплечный             правая стрелка                левая стрелка
      LedShow(StripRight.Color(0, 0, 0), StripLeft.Color(255, 100, 0), StripBack.Color(0, 0, 0), StripBack.Color(255, 100, 0));
      Serial.println("LED Left ON");
    }
    // Выключаем LED на ... секунд
    else if (!blinkFix  && (millis() >= tmrOn)) {
      tmrOff = millis() + 200;
      blinkFix = !blinkFix;
      //          правый наплечный          левый наплечный           правая стрелка              левая стрелка
      LedShow(StripRight.Color(0, 0, 0), StripLeft.Color(0, 0, 0), StripBack.Color(0, 0, 0), StripBack.Color(0, 0, 0));
      Serial.println("LED Left OFF");
    }
  }
  /////////////////////////////////////////////////////////////////
  // Включаем СТОП
  else if ((RouteFix == 'S') && (TimeFix > 0)) {
    //          правый наплечный          левый наплечный           правая стрелка              левая стрелка
    LedShow(StripRight.Color(0, 0, 0), StripLeft.Color(0, 0, 0), StripBack.Color(255, 0, 0), StripBack.Color(255, 0, 0));
    Serial.println("LED Stop On");
  }
  /////////////////////////////////////////////////////////////////
  // Выключаем ВСЁ
  else if (TimeFix < 1) {
    RouteFix = 'N';
    TimeFix = 0;
    LedShow(StripRight.Color(0, 0, 0), StripLeft.Color(0, 0, 0), StripBack.Color(0, 0, 0), StripBack.Color(0, 0, 0));
    Serial.println("LED OFF");
  }

  if (RouteFix == 'G')  {
    for (int i = 0; i <= 13; i++) {
      StripGlim.setPixelColor(i, StripGlim.Color(255, 0, 0));
    }
    StripGlim.show();
    Serial.println("LED Glim On");
  }
  else if (RouteFix == 'F') {
    for (int i = 0; i <= 13; i++) {
      StripGlim.setPixelColor(i, StripGlim.Color(0, 0, 0));
    }
    StripGlim.show();
    Serial.println("LED Glim Off");
  }
}

//colorRinght - правые наплечные LED
//colorLeft - левые наплечные LED
//colorBackRinght - правая стрелка сзади
//colorBackLeft - левая стрелка сзади
void LedShow(uint32_t colorRinght, uint32_t colorLeft, uint32_t colorBackRinght, uint32_t colorBackLeft) {
  for (int i = 0; i <= 13; i++) {
    StripRight.setPixelColor(i, colorRinght);
    StripLeft.setPixelColor(i, colorLeft);
    StripBack.setPixelColor(i, colorBackLeft);
    StripBack.setPixelColor(i + 14 , colorBackRinght);
  }
  StripRight.show();
  StripLeft.show();
  StripBack.show();
  //   Serial.print("++++");
  //   Serial.print("\t");
}
