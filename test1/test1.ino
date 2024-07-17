#define BLYNK_TEMPLATE_ID "TMPL3D-mYac_r"
#define BLYNK_TEMPLATE_NAME "Motor"
#define BLYNK_AUTH_TOKEN "JTcLBwpeIz65KDvFa7MW3l8WeKNrBwUE"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#define Current_sensor 34
#define Out1 21
#define Out2 17
#define Out1_2_speed 22

int ledchannel = 0;
int freq = 5000;
int resolution = 8;
int mVperAmp = 185;           // this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

char ssid[] = "SAIF_PC";
char pass[] = "a1234567";

void setup() {
  Serial.begin(115200); 
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  pinMode(Out1, OUTPUT);
  pinMode(Out2, OUTPUT);
  pinMode(Out1_2_speed, OUTPUT);
  ledcSetup(ledchannel, freq, resolution);
  ledcAttachPin(Out1_2_speed,ledchannel);
  ledcWrite(ledchannel,0);
  digitalWrite(Out1, LOW);
  digitalWrite(Out2, LOW);
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

void get_current(){
  Voltage = getVPP();
  VRMS = (Voltage/2.0) *0.707;   //root 2 is 0.707
  AmpsRMS = ((VRMS * 1000)/mVperAmp)-0.3; //0.3 is the error I got for my sensor
 
  Serial.print(AmpsRMS);
  Serial.print(" Amps RMS  ---  ");
  Watt = (AmpsRMS*240/1.2);
  // note: 1.2 is my own empirically established calibration factor
// as the voltage measured at D34 depends on the length of the OUT-to-D34 wire
// 240 is the main AC power voltage – this parameter changes locally
  Serial.print(Watt);
  Serial.println(" Watts");
  delay(50);
}

void motor_speed_control(int speed, int direction){
  ledcWrite(ledchannel,speed);
  if(direction == 1){
    digitalWrite(Out1, HIGH);
    digitalWrite(Out2, LOW);
  }
  else{
    digitalWrite(Out1, LOW);
    digitalWrite(Out2, HIGH);
  }
}

BLYNK_WRITE(V0)
{
  int fanspeed = param.asInt();
  ledcWrite(ledchannel,fanspeed);
}
BLYNK_WRITE(V1)
{
   
    int pinvalue = param.asInt();
    if(pinvalue==1)
    {
      digitalWrite(Out1, HIGH);
      digitalWrite(Out2, LOW);
      Blynk.virtualWrite(V1, HIGH);
    }
    else
    {
      digitalWrite(Out1, LOW);
      digitalWrite(Out2, HIGH);
      Blynk.virtualWrite(V1, LOW);
    }
        
}
BLYNK_WRITE(V2)
{
   
    int pinvalue = param.asInt();
    if(pinvalue==1)
    {
      Blynk.virtualWrite(V1, HIGH);
    }
    else
    {
      digitalWrite(Out1, LOW);
      digitalWrite(Out2, LOW);
    }
        
}

void loop() {
  get_current();
  Blynk.run();
  //motor_speed_control(255, 1);
}