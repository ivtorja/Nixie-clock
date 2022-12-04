
// NO SE RESETEA A CERO EL ENCODER EN CADA CAMBIO
// CALIBRAR TEMPERATURA
// AÑADIR RECONEXION WIFI

#include <ESP32Time.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <Adafruit_NeoPixel.h>
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define SEGUNDOS1 32
#define SEGUNDOS2 15
#define MINUTOS1 33
#define MINUTOS2 27
#define HORAS1 12
#define HORAS2 13

#define DOTS A0

#define DRIVER_A 21
#define DRIVER_B 17
#define DRIVER_C 16
#define DRIVER_D 19

#define ROTARY_ENCODER_BUTTON_PIN 14
#define ROTARY_ENCODER_A_PIN A4
#define ROTARY_ENCODER_B_PIN A3

#define LED_PIN    A5
#define LED_COUNT 6

#define LDR A2

#define TEMPERATURA 5

const int oneWireBus = TEMPERATURA;
OneWire oneWire(oneWireBus);
DallasTemperature sensor_temperatura(&oneWire);

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
ESP32Time rtc;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// USER SETTINGS
const char* ssid1 = "Nixie";  // Your WiFi network
const char* password1 = "1234567890";  // Your WiFi password

const char* ssid2 = "SSID";  // Your WiFi network
const char* password2 = "PASSWORD";  // Your WiFi password

const char* ntpServer = "hora.roa.es";
const long gmtOffset_sec = 1*3600; //change this to adapt it to your time zone
const int daylightOffset_sec = 1*3600;
// END USER SETTINGS

int brightness;
int max_brightness=255;

int hours, mins, secs, day, month, year;
int vector_anodos [6] = {HORAS2, HORAS1, MINUTOS2, MINUTOS1, SEGUNDOS2, SEGUNDOS1};

int vector_digitos [10][4] = {
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1},
  {0, 0, 0, 0},
  {1, 0, 0, 0},
  {0, 0, 0, 1},
  {0, 1, 0, 0},
  {0, 1, 1, 1}
};

int mydigit [6] = {0, 0, 0, 0, 0, 0};
int digit_begin = 0;
bool dot_status = 0;
volatile int interruptCounter = 0;
bool apagado_noche = 0;
int temperatura;

#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 20

//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

WiFiMulti wifiMulti;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  if (!apagado_noche)
  {
    for (int digit = digit_begin; digit < 6; digit++) {
      BCDout(mydigit[digit]);
      digitalWrite(vector_anodos[digit], HIGH);
      delayMicroseconds(map(brightness, 0, max_brightness, 500, 3000)); //2500
      digitalWrite(vector_anodos[digit], LOW);
    }
  }
  else
  {
    if (rotaryEncoder.isEncoderButtonClicked())
    {
      apagado_noche=0;
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

void setup() {
  Serial.begin(115200);
  sensor_temperatura.begin();

  pinMode(LDR, INPUT);
  pinMode(SEGUNDOS1, OUTPUT);
  pinMode(SEGUNDOS2, OUTPUT);
  pinMode(MINUTOS1, OUTPUT);
  pinMode(MINUTOS2, OUTPUT);
  pinMode(HORAS1, OUTPUT);
  pinMode(HORAS2, OUTPUT);
  pinMode(DRIVER_A, OUTPUT);
  pinMode(DRIVER_B, OUTPUT);
  pinMode(DRIVER_C, OUTPUT);
  pinMode(DRIVER_D, OUTPUT);

  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setEncoderValue(0);
  bool circleValues = true;
  rotaryEncoder.setAcceleration(0); //250

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(map(brightness, 0, 255, 10, 100));
  
  wifiMulti.addAP(ssid1, password1);
  wifiMulti.addAP(ssid2, password2);
  int attempts = 0;
  
  Serial.println("Conectando al WiFi");
  long st = millis();
  while (wifiMulti.run() != WL_CONNECTED) {
    if (++attempts < 20 && !rotaryEncoder.isEncoderButtonClicked()) {
      colorWipe(strip.Color(3,   165,  252), 200); // light blue
      strip.clear();
    } else
    {
      colorWipe(strip.Color(255, 0, 0), 0);
      break;
    }
  }

  ledcAttachPin(DOTS, 0); // assign a led pins to a channel
  ledcSetup(0, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcWrite(0, 0);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 20000, true);
  timerAlarmEnable(timer);

  if (WiFi.status() == WL_CONNECTED) {
    long tm = millis() - st;
    Serial.print(F("\nConnected to AP in "));
    Serial.print(tm, DEC);
    Serial.print(F(" ms. IP: "));
    for (byte thisByte = 0; thisByte < 4; thisByte++) {
      Serial.print(String(WiFi.localIP()[thisByte], DEC));
      if (thisByte != 3) Serial.print(".");
    }
    Serial.println();
    setNTP();
    get_time();
  }
  else
  {
    WiFi.disconnect();
    Serial.println("\nNO SE HA PODIDO CONECTAR");
    Serial.println("Manual time setting");
    int horas, minutos, segundos;
    Serial.println("HORAS");
    while (!rotaryEncoder.isEncoderButtonClicked())
    {
      rotaryEncoder.setBoundaries(0, 23, circleValues);
      if (rotaryEncoder.encoderChanged())
      {
        horas = rotaryEncoder.readEncoder();
        mydigit[0] = rotaryEncoder.readEncoder() / 10;
        mydigit[1] = rotaryEncoder.readEncoder() % 10;

      }
    }
    Serial.println("MINUTOS");
    while (!rotaryEncoder.isEncoderButtonClicked())
    {
      rotaryEncoder.setBoundaries(0, 59, circleValues);
      if (rotaryEncoder.encoderChanged())
      {
        minutos = rotaryEncoder.readEncoder();
        mydigit[2] = rotaryEncoder.readEncoder() / 10;
        mydigit[3] = rotaryEncoder.readEncoder() % 10;
      }
    }
    Serial.println("SEGUNDOS");
    while (!rotaryEncoder.isEncoderButtonClicked())
    {
      rotaryEncoder.setBoundaries(0, 59, circleValues);
      if (rotaryEncoder.encoderChanged())
      {
        segundos = rotaryEncoder.readEncoder();
        mydigit[4] = rotaryEncoder.readEncoder() / 10;
        mydigit[5] = rotaryEncoder.readEncoder() % 10;
      }
    }
    rtc.setTime(segundos, minutos, horas, 1, 1, 2022);
    get_time();
    mydigit[0] = hours / 10;
    mydigit[1] = hours % 10;
    mydigit[2] = mins / 10;
    mydigit[3] = mins % 10;
    mydigit[4] = secs / 10;
    mydigit[5] = secs % 10;
  }
  colorWipe(strip.Color(3,   165,  252), 0); // light blue
  Serial.println(rtc.getTime("%d/%m/%Y %H:%M:%S"));
}

void loop() {
  secs = rtc.getSecond();
  if (secs == 0) get_time();
  mydigit[0] = hours / 10;
  mydigit[1] = hours % 10;
  mydigit[2] = mins / 10;
  mydigit[3] = mins % 10;
  mydigit[4] = secs / 10;
  mydigit[5] = secs % 10;


  if (interruptCounter > 19) {
    interruptCounter = 0;
    dot_status = !dot_status;
    if (dot_status && !apagado_noche)
    {
      ledcWrite(0, constrain(brightness, 40, 255));
    }
    else
    {
      ledcWrite(0, 0);
    }
  }

  while (secs == rtc.getSecond())delay(10); // wait until seconds change
  if (mins == 59 && secs == 59)
  {
    if (WiFi.status() == WL_CONNECTED) setNTP(); // get NTP time every hour if WiFi connected
    // SLOT MACHINE
    for (int veces = 0; veces < 3; veces++)
    {
      for (int numero = 0; numero < 10; numero++)
      {
        mydigit[0] = numero;
        mydigit[1] = numero;
        mydigit[2] = numero;
        mydigit[3] = numero;
        mydigit[4] = numero;
        mydigit[5] = numero;
        delay(100);
      }
    }
    // Serial.println(rtc.getTime("%d/%m/%Y %H:%M:%S"));
    get_time();
    mydigit[0] = hours / 10;
    mydigit[1] = hours % 10;
    mydigit[2] = mins / 10;
    mydigit[3] = mins % 10;
    mydigit[4] = secs / 10;
    mydigit[5] = secs % 10;
  }
  
  if (hours<18 && hours>8) max_brightness = 255;
  else max_brightness = 200;
  brightness = map(analogRead(LDR), 0, 3400, 0, max_brightness);
  strip.setBrightness(map(brightness, 0, max_brightness, 10, 100));
  strip.show();

  if (rotaryEncoder.isEncoderButtonClicked())
  {
    sensor_temperatura.requestTemperatures();
    temperatura = (int)sensor_temperatura.getTempCByIndex(0);
    ledcWrite(0, 0);
    while (rtc.getSecond() - secs < 5)
    {
      digit_begin = 4;
      mydigit[4] = temperatura / 10;
      mydigit[5] = temperatura % 10;
    }
    digit_begin = 0;
  }
}


void BCDout(int mynumber) {  // this routine take a number and outputs it to the 74141 as BCD (Binary Coded Decimal)
  if (mynumber > -1 && mynumber < 11) {  // make sure number is in range
    delayMicroseconds(100);
    digitalWrite(DRIVER_D, vector_digitos[mynumber][0]);
    digitalWrite(DRIVER_C, vector_digitos[mynumber][1]);
    digitalWrite(DRIVER_B, vector_digitos[mynumber][2]);
    digitalWrite(DRIVER_A, vector_digitos[mynumber][3]);
  }
}

void setNTP() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  //Serial.print("DST: ");
  //Serial.println(timeinfo.tm_isdst);
  if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo);
  }
}

void get_time() {
  mins = rtc.getMinute();
  hours = rtc.getHour(true);
  day = rtc.getDay();
  month = rtc.getMonth();
  year = rtc.getYear();

  if (hours > 0 && hours < 7)
  {
    apagado_noche = 1;
    strip.clear();
  }
  else
  {
    apagado_noche = 0;
    colorWipe(strip.Color(3,   165,  252), 0); // light blue
  }
}


void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}
