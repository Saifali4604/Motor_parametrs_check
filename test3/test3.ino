#define BLYNK_TEMPLATE_ID "TMPL3D-mYac_r"
#define BLYNK_TEMPLATE_NAME "Motor"
#define BLYNK_AUTH_TOKEN "JTcLBwpeIz65KDvFa7MW3l8WeKNrBwUE"
#define BLYNK_PRINT Serial
#include <Arduino.h>
#include <driver/i2s.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>             // Arduino SPI library
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "image.h"

#define TFT_CS  13
#define TFT_RST 5
#define TFT_DC 4
#define Current_sensor 34
#define Out1 21
#define Out2 17
#define Out1_2_speed 22
#define D2 2
#define ENCODER_PIN 27
#define ENCODER_N   20

long unsigned int Time1 = 0, Time2 = 0, TimeInterval = 0;
volatile bool MeasurementDone = false;
int Motor_RPM = 0;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int ledchannel = 0;
int freq = 5000;
int resolution = 8;
int mVperAmp = 185;
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

char ssid[] = "SAIF_PC";
char pass[] = "a1234567";

SemaphoreHandle_t tftMutex;

void display_Amp(void *pvParameters);
TaskHandle_t display_Amp_handler;

void display_RPM(void *pvParameters);
TaskHandle_t display_RPM_handler;

void IRAM_ATTR encoderISR() {
  if (MeasurementDone) {
    Time2 = micros();
    TimeInterval = Time2 - Time1;
    MeasurementDone = false;
  } else {
    Time1 = micros();
    MeasurementDone = true;
  }
}

void setup() {
  Serial.begin(115200);
  tft.init(240, 240, SPI_MODE2);
  tft.setRotation(3);
  tft.fillRect(0, 0, 240, 240, ST77XX_BLACK);
  tft.drawBitmap(0, 0, RV_logo, 250, 250, ST77XX_WHITE);
  delay(2000);
  tft.fillRect(0, 0, 240, 240, ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_BLUE);
  tft.setCursor(20, 50);
  tft.println(" Connecting");
  tft.setCursor(20, 100);
  tft.println("     to    ");
  tft.setCursor(60, 150);
  tft.println(ssid);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  pinMode(Out1, OUTPUT);
  pinMode(Out2, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(Out1_2_speed, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
  ledcSetup(ledchannel, freq, resolution);
  ledcAttachPin(Out1_2_speed, ledchannel);
  ledcWrite(ledchannel, 0);
  digitalWrite(Out1, LOW);
  digitalWrite(Out2, LOW);
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0, 0, 240, 240, ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 10);
  tft.println("Current :");

  tftMutex = xSemaphoreCreateMutex();

  xTaskCreate(display_Amp, "display_Amp", 2048, NULL, 1, &display_Amp_handler);
  xTaskCreate(display_RPM, "display_RPM", 2048, NULL, 1, &display_RPM_handler);
}

float getVPP() {
  float result;
  int readValue;
  int maxValue = 0;
  int minValue = 4096;

  uint32_t start_time = millis();
  while ((millis() - start_time) < 1000) {
    readValue = analogRead(Current_sensor);
    Serial.println(readValue);
    if (readValue > maxValue) {
      maxValue = readValue;
    }
    if (readValue < minValue) {
      minValue = readValue;
    }
  }
  result = ((maxValue - minValue) * 3.3) / 4096.0;
  return result;
}

float getStableCurrent() {
  const int numSamples = 5;
  float totalCurrent = 0;
  for (int i = 0; i < numSamples; i++) {
    float voltage = getVPP();
    float vrms = (voltage / 2.0) * 0.707;
    float ampsRMS = ((vrms * 1000) / mVperAmp) - 0.04;
    totalCurrent += ampsRMS;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  return totalCurrent / numSamples;
}

void display_Amp(void *pvParameters) {
  for (;;) {
    AmpsRMS = getStableCurrent();

    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
      tft.fillRect(10, 50, 220, 60, ST77XX_BLACK);
      tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
      tft.setTextSize(4);
      tft.setCursor(20, 50);
      tft.print(AmpsRMS);
      tft.setTextSize(4);
      tft.setCursor(160, 50);
      tft.println("A");
      xSemaphoreGive(tftMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

float getAverageRPM() {
  const int numSamples = 10;
  long totalRPM = 0;
  for (int i = 0; i < numSamples; i++) {
    if (TimeInterval > 0) {
      totalRPM += (60000000 / (TimeInterval * ENCODER_N));
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return totalRPM / numSamples;
}

void display_RPM(void *pvParameters) {
  for (;;) {
    Motor_RPM = getAverageRPM();

    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
      tft.fillRect(10, 110, 220, 60, ST77XX_BLACK);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setTextSize(4);
      tft.setCursor(20, 110);
      tft.print(Motor_RPM);
      tft.setTextSize(4);
      tft.setCursor(160, 110);
      tft.println("RPM");
      xSemaphoreGive(tftMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void loop() {
  Blynk.run();
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(D2, LOW);
  } else {
    digitalWrite(D2, HIGH);
  }
}

BLYNK_WRITE(V0)
{
  int fanspeed = param.asInt();
  ledcWrite(ledchannel,fanspeed);
}

BLYNK_WRITE(V2)
{
   
    int pin2value = param.asInt();
    if(pin2value==1)
    {
      //Blynk.virtualWrite(V1, HIGH);
    }
    else
    {
      digitalWrite(Out1, LOW);
      digitalWrite(Out2, LOW);
    }
        
}
BLYNK_WRITE(V1) {
  int pin1value = param.asInt();
  if (pin1value == 1) {
    digitalWrite(Out1, HIGH);
    digitalWrite(Out2, LOW);
    Blynk.virtualWrite(V1, HIGH);
    Serial.print("clock");
  } else if (pin1value == 0) {
    digitalWrite(Out1, LOW);
    digitalWrite(Out2, HIGH);
    Blynk.virtualWrite(V1, LOW);
    Serial.print("anticlock");
  }
}
