#ifndef ESP32
#error This code is designed to run on ESP32 platform, not Arduino nor ESP8266! Please check your Tools->Board setting.
#endif

#define BLYNK_PRINT Serial

//See file .../hardware/espressif/esp32/variants/(esp32|doitESP32devkitV1)/pins_arduino.h
#define LED_BUILTIN       2         // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED
#define PIN_LED           2         // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED

#define PIN_D0            0         // Pin D0 mapped to pin GPIO0/BOOT/ADC11/TOUCH1 of ESP32
#define PIN_D1            1         // Pin D1 mapped to pin GPIO1/TX0 of ESP32
#define PIN_D2            2         // Pin D2 mapped to pin GPIO2/ADC12/TOUCH2 of ESP32
#define PIN_D3            3         // Pin D3 mapped to pin GPIO3/RX0 of ESP32
#define PIN_D4            4         // Pin D4 mapped to pin GPIO4/ADC10/TOUCH0 of ESP32
#define PIN_D5            5         // Pin D5 mapped to pin GPIO5/SPISS/VSPI_SS of ESP32
#define PIN_D6            6         // Pin D6 mapped to pin GPIO6/FLASH_SCK of ESP32
#define PIN_D7            7         // Pin D7 mapped to pin GPIO7/FLASH_D0 of ESP32
#define PIN_D8            8         // Pin D8 mapped to pin GPIO8/FLASH_D1 of ESP32
#define PIN_D9            9         // Pin D9 mapped to pin GPIO9/FLASH_D2 of ESP32
  
#define PIN_D10           10        // Pin D10 mapped to pin GPIO10/FLASH_D3 of ESP32
#define PIN_D11           11        // Pin D11 mapped to pin GPIO11/FLASH_CMD of ESP32
#define PIN_D12           12        // Pin D12 mapped to pin GPIO12/HSPI_MISO/ADC15/TOUCH5/TDI of ESP32
#define PIN_D13           13        // Pin D13 mapped to pin GPIO13/HSPI_MOSI/ADC14/TOUCH4/TCK of ESP32
#define PIN_D14           14        // Pin D14 mapped to pin GPIO14/HSPI_SCK/ADC16/TOUCH6/TMS of ESP32
#define PIN_D15           15        // Pin D15 mapped to pin GPIO15/HSPI_SS/ADC13/TOUCH3/TDO of ESP32
#define PIN_D16           16        // Pin D16 mapped to pin GPIO16/TX2 of ESP32
#define PIN_D17           17        // Pin D17 mapped to pin GPIO17/RX2 of ESP32     
#define PIN_D18           18        // Pin D18 mapped to pin GPIO18/VSPI_SCK of ESP32
#define PIN_D19           19        // Pin D19 mapped to pin GPIO19/VSPI_MISO of ESP32

#define PIN_D21           21        // Pin D21 mapped to pin GPIO21/SDA of ESP32
#define PIN_D22           22        // Pin D22 mapped to pin GPIO22/SCL of ESP32
#define PIN_D23           23        // Pin D23 mapped to pin GPIO23/VSPI_MOSI of ESP32
#define PIN_D24           24        // Pin D24 mapped to pin GPIO24 of ESP32
#define PIN_D25           25        // Pin D25 mapped to pin GPIO25/ADC18/DAC1 of ESP32
#define PIN_D26           26        // Pin D26 mapped to pin GPIO26/ADC19/DAC2 of ESP32
#define PIN_D27           27        // Pin D27 mapped to pin GPIO27/ADC17/TOUCH7 of ESP32     
   
#define PIN_D32           32        // Pin D32 mapped to pin GPIO32/ADC4/TOUCH9 of ESP32
#define PIN_D33           33        // Pin D33 mapped to pin GPIO33/ADC5/TOUCH8 of ESP32
#define PIN_D34           34        // Pin D34 mapped to pin GPIO34/ADC6 of ESP32
#define PIN_D35           35        // Pin D35 mapped to pin GPIO35/ADC7 of ESP32
#define PIN_D36           36        // Pin D36 mapped to pin GPIO36/ADC0/SVP of ESP32
#define PIN_D39           39        // Pin D39 mapped to pin GPIO39/ADC3/SVN of ESP32

#define PIN_RX0            3        // Pin RX0 mapped to pin GPIO3/RX0 of ESP32
#define PIN_TX0            1        // Pin TX0 mapped to pin GPIO1/TX0 of ESP32

#define PIN_SCL           22        // Pin SCL mapped to pin GPIO22/SCL of ESP32
#define PIN_SDA           21        // Pin SDA mapped to pin GPIO21/SDA of ESP32  

#define USE_SPIFFS    true
//#define USE_SPIFFS    false

//#define USE_BLYNK_WM    true            // https://github.com/khoih-prog/Blynk_WM
#define USE_BLYNK_WM    false

//LIBRARIES INCLUDED
#include <WiFi.h>
#include <WiFiClient.h>

#if USE_BLYNK_WM
  #include <BlynkSimpleEsp32_WM.h>                    // https://github.com/khoih-prog/Blynk_WM
#else
  #include <BlynkSimpleEsp32.h>

  //BLYNK AUTHENTICATION TOKEN
  char auth[] = "******";
  
  
  // MY WIFI CREDENTIALS
  char ssid[] = "****";
  char pass[] = "****";
  
#endif
  
#include <DHT.h>

#define USE_ESP32_ISR_SERVO       true

#if USE_ESP32_ISR_SERVO

  #define TIMER_INTERRUPT_DEBUG       1
  #define ISR_SERVO_DEBUG             1
  
  // Select different ESP32 timer number (0-3) to avoid conflict
  #define USE_ESP32_TIMER_NO          3

  #include "ESP32_ISR_Servo.h"

  // MG996R servo has a running current of  500mA to 900mA @6V and a stall current of 2.5A @ 6V
  // Power supply must be adequate
  // Published values for SG90 servos; adjust if needed
  #define MIN_MICROS      800  //544
  #define MAX_MICROS      2450
  
  int servoIndex1  = -1;
  int servoIndex2  = -1;
  int servoIndex3  = -1;

  int servo1Pin = PIN_D25; //SERVO1 PIN
  int servo2Pin = PIN_D26; //SERVO2 PIN
  int servo3Pin = PIN_D27; //SERVO3 PIN
    
#else

  #include <ESP32Servo.h>

  //SERVO INSTANCE
  Servo servo1; //SERVO1
  Servo servo2; //SERVO2
  Servo servo3; //SERVO3
  
  int minUs = 1000;
  int maxUs = 2000;

  int servo1Pin = 18; //SERVO1 PIN
  int servo2Pin = 19; //SERVO2 PIN
  int servo3Pin = 21; //SERVO3 PIN

  ESP32PWM pwm;
  
#endif


// DHT INSTANCE
#define DHTPIN 27
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

BlynkTimer timer;

//DECLARATION OF VARIABLES
#define rain 32     
#define ldr 33
#define gas 35
#define pir 26
#define buzzer 5
int pir_value;
int pir_check;
int ldr_value;
int rain_value;
int gas_value;
bool Connected2Blynk = false;

//FUNCTIONS

void send_values()
{
  sendrain_value();
  sendldr_value();
  sendgas_value();
  sendtemp_value();
  
}

//DHT TEMPERATURE
void sendtemp_value()
{
  float temp_value = dht.readTemperature();
  Blynk.virtualWrite(V1, temp_value);
}

//RAIN
void sendrain_value()
{
  rain_value = analogRead(rain);
  rain_value = map(rain_value, 1, 4095, 1, 100);
  Blynk.virtualWrite(V2, rain_value);
}

//LDR
void sendldr_value()
{
  ldr_value = analogRead(ldr);
  ldr_value = map(ldr_value, 0, 4095, 0, 100);
  Blynk.virtualWrite(V3, ldr_value);
}

//ULTRASONIC
void sendgas_value()
{
  gas_value = analogRead(gas);
  Blynk.virtualWrite(V4, gas_value);
}

//PIR
BLYNK_WRITE(V5)
{
  pir_check = param.asInt();
  Serial.print("PIR CHECK: ");
  Serial.println(pir_check);
} 


//READING FROM VIRTUAL PINS
// SERVO1
BLYNK_WRITE(V6)
{
  #if USE_ESP32_ISR_SERVO
    ESP32_ISR_Servos.setPosition(servoIndex1, param.asInt());
  #else
    // Too much work inside BLYNK_WRITE()
    servo1.attach(servo1Pin, minUs, maxUs);
    servo1.write(param.asInt());
    servo1.detach();
  #endif
}

//SERVO2
BLYNK_WRITE(V7)
{
  #if USE_ESP32_ISR_SERVO
    ESP32_ISR_Servos.setPosition(servoIndex2, param.asInt());
  #else
    // Too much work inside BLYNK_WRITE()
    servo2.attach(servo2Pin, minUs, maxUs);
    servo2.write(param.asInt());
    servo2.detach();
  #endif  
}

//SERVO3
BLYNK_WRITE(V8)
{
  #if USE_ESP32_ISR_SERVO
    ESP32_ISR_Servos.setPosition(servoIndex3, param.asInt());
  #else
    // Too much work inside BLYNK_WRITE()
    servo3.attach(servo3Pin, minUs, maxUs);
    servo3.write(param.asInt());
    servo3.detach();
  #endif  
}

//PIR Alarm
void pir_alarm()
{
  if (pir_check == 1)
  {
    pir_value = digitalRead(pir);
    Serial.print("PIR VALUE: ");
    Serial.println(pir_value);
    if (pir_value == 1)
    {
      digitalWrite(buzzer, HIGH);
    }
    else
    {
      digitalWrite(buzzer, LOW);
    }
  }
}

//VOID SETUP
void setup()
{
  // Debug console
  Serial.begin(115200);
  Serial.println("\nStarting");
  
  dht.begin();

  #if USE_BLYNK_WM
    Blynk.begin();
  #else
    Blynk.begin(auth, ssid, pass);
  #endif

  // 1s timer to check PIR alarm
  timer.setInterval(1000, pir_alarm);
  
  // Change to 10s timer
  timer.setInterval(10000, send_values);
  //timer.setInterval(1000, sendrain_value);
  //timer.setInterval(1000, sendldr_value);
  //timer.setInterval(1000, sendgas_value);
  //timer.setInterval(1000, sendtemp_value);
  
  pinMode(pir, INPUT);
  pinMode(rain, INPUT);
  pinMode(ldr, INPUT);
  pinMode(gas, INPUT);
  pinMode(buzzer, OUTPUT);

  #if USE_ESP32_ISR_SERVO
    //Select ESP32 timer USE_ESP32_TIMER_NO
    ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);
    
    servoIndex1 = ESP32_ISR_Servos.setupServo(servo1Pin, MIN_MICROS, MAX_MICROS);
    servoIndex2 = ESP32_ISR_Servos.setupServo(servo2Pin, MIN_MICROS, MAX_MICROS);
    servoIndex3 = ESP32_ISR_Servos.setupServo(servo3Pin, MIN_MICROS, MAX_MICROS);
    
    if (servoIndex1 != -1)
      Serial.println("Setup Servo1 OK");
    else
      Serial.println("Setup Servo1 failed");
  
    if (servoIndex2 != -1)
      Serial.println("Setup Servo2 OK");
    else
      Serial.println("Setup Servo2 failed");    

    if (servoIndex3 != -1)
      Serial.println("Setup Servo3 OK");
    else
      Serial.println("Setup Servo3 failed");    

  #else
    servo1.setPeriodHertz(50);      
    servo2.setPeriodHertz(50);     
    servo3.setPeriodHertz(330);
  #endif
}

//VOID LOOP RUNS CONTINUOUSLY THROUGH OUT THE PROGRAM
void loop()
{
  Blynk.run();
  timer.run();

#if 0
  //EXTRA CODE FOR PIR SHOULD BE REMOVED FROM LOOP
  if (pir_check == 1)
  {
    pir_value = digitalRead(pir);
    Serial.print("PIR VALUE: ");
    Serial.println(pir_value);
    if (pir_value == 1)
    {
      digitalWrite(buzzer, HIGH);
    }
    else
    {
      digitalWrite(buzzer, LOW);
    }
  }
#endif
}
