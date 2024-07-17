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
#define I2S_WS 15    // I2S word select pin
#define I2S_SCK 14   // I2S clock pin
#define I2S_SD 32    // I2S data pin
#define Current_sensor 34
#define Out1 21
#define Out2 17
#define Out1_2_speed 22
#define D2 2

// Initialize Adafruit ST7789 TFT library
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

// Mutex for TFT display access
SemaphoreHandle_t tftMutex ;

void display_Amp(void *pvParameters); 
TaskHandle_t display_Amp_handler; 

void setup(){ 
  Serial.begin(115200);
  tft.init(240, 240, SPI_MODE2);    // Init ST7789 display 240x240 pixels
  tft.setRotation(3);
  tft.drawBitmap(0,0,RV_logo,250,250,ST77XX_WHITE);
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
  ledcSetup(ledchannel, freq, resolution);
  ledcAttachPin(Out1_2_speed,ledchannel);
  ledcWrite(ledchannel,0);
  digitalWrite(Out1, LOW);
  digitalWrite(Out2, LOW);
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0, 0, 240, 240, ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 10);
  tft.println("Current :");

   // Create mutex for TFT display access
  tftMutex = xSemaphoreCreateMutex();

  xTaskCreate(display_Amp, "display", 2048, NULL, 1 , &display_Amp_handler);
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

float getVPP()
{
  float result;
  int readValue;                // value read from the sensor
  int maxValue = 0;             // store max value here
  int minValue = 4096;          // store min value here ESP32 ADC resolution
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(Current_sensor);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the minimum sensor value*/
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * 3.3)/4096.0; //ESP32 ADC resolution 4096
      
   return result;
}
void display_Amp(void *pvParameters){  
  for(;;){
    Voltage = getVPP();
    VRMS = (Voltage/2.0) *0.707;   //root 2 is 0.707
    AmpsRMS = ((VRMS * 1000)/mVperAmp)-0.04; //0.06 is the error I got for my sensor
  
    // Serial.print(AmpsRMS);
    // Serial.println(" Amps RMS  ---  ");
    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
    tft.fillRect(10, 50, 220, 60, ST77XX_BLACK);
    tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    tft.setTextSize(4);
    tft.setCursor(20, 50);
    tft.print(AmpsRMS);
    tft.setTextSize(4);
    tft.setCursor(160, 50); // Adjusted Speed text position
    tft.println("A");
    vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreGive(tftMutex);
  }
  }
}


void loop(){
  Blynk.run();
  if(WiFi.status() != WL_CONNECTED)
    {
     digitalWrite(D2, LOW);
     }
    else
    {
      digitalWrite(D2, HIGH);
    }
} 
BLYNK_WRITE(V1)
{
   
    int pin1value = param.asInt();
    if(pin1value==1)
    {
      digitalWrite(Out1, HIGH);
      digitalWrite(Out2, LOW);
      Blynk.virtualWrite(V1, HIGH);
      Serial.print("clock");
    }
    else if(pin1value==0)
    {
      digitalWrite(Out1, LOW);
      digitalWrite(Out2, HIGH);
      Blynk.virtualWrite(V1, LOW);
      Serial.print("anticlock");
    }
        
}
