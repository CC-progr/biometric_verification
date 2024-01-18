#include "Wire.h"
#include <LiquidCrystal_I2C.h>
#include <string>
#include <iostream>
#include <cstdlib>
#include <ESP32Servo.h>
 
#define WIRE_SPEED 3400000    

#define MCPADDRESS 0x20 // Conf LOW-LOW-LOW

// Main registers
#define IODIRA 0x00
#define IODIRB 0x01
#define IOCON  0x05
#define GPIOA  0x12
#define GPIOB  0x13
#define GPPUA  0x0C
#define GPPUB  0x0D

// Control
#define BANK   0x80 // 0b10000000 --> 1 Different banks, 0 Same bank
#define MIRROR 0x40 // 0b01000000 --> 1 Int connected, 0 Int dissociated (A/B)
#define SEQOP  0x20 // 0b00100000 --> 1 SEQ, 0 DIRECT
#define DISSLW 0x10 // 0b00010000 --> 1 Disabled
#define HAEN   0x08 // 0b00001000 --> 1 HW , 0 Disabled
#define ODR    0x04 // 0b00000100 --> 1 Open Drain
#define INTPOL 0x02 // 0b00000010 --> 1 INT Active High, INT 0 Active Low

// uint16_t time1, time2;
LiquidCrystal_I2C lcd(0x27,16,2);


//freeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <WiFi.h>
#include <PubSubClient.h>

// #define WIFI_SSID "caca2"
// #define WIFI_PASSWORD "cacacaca"

// #define WIFI_SSID "PapitaC"
// #define WIFI_PASSWORD "wpgr3523"

#define WIFI_SSID "Ruben"
#define WIFI_PASSWORD "123456789"

#define MQTT_USER "user1"
#define MQTT_PASS "1234"
#define MQTT_BROKER "192.168.43.105"
#define MQTT_PORT 1883

#define SENSOR1_TOPIC "/eps-L1/location/sensor1"
#define SENSOR2_TOPIC "/eps-L1/location/sensor2"

#define USER_HR_TOPIC "/eps-L1/user/HR"
#define USER_GSR_TOPIC "/eps-L1/user/GSR"
#define USER_PREDICTION_TOPIC "/eps-L1/user/prediction"

SemaphoreHandle_t i2cMutex;

SemaphoreHandle_t sensor1Mutex;
SemaphoreHandle_t sensor2Mutex;
long sensor1;
long sensor2;

SemaphoreHandle_t gsrMutex;
SemaphoreHandle_t hrMutex;
SemaphoreHandle_t usernameMutex;
String gsr = "0";
String hr = "0";
String username = "unknown";

Servo miServo;
const int pinServo = 23;

WiFiClient espClient;
PubSubClient client;

void servoTask(void* pvParameters);
void wifiTask(void* pvParameters);
void ledTask(void* pvParameters);
void ledFarLeft();
void ledCloseLeft();
void ledCenter();
void ledCloseRight();
void ledFarRight();
void lcdTask(void* pvParameters);


void setup(){
  Serial.begin(115200);
  Wire.setClock(WIRE_SPEED);
  Wire.begin();

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(IOCON);
  Wire.write(SEQOP | MIRROR | HAEN );
  Wire.endTransmission();

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(IODIRA);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPPUA);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  

  // Create the semaphores
  i2cMutex = xSemaphoreCreateMutex();
  sensor1Mutex = xSemaphoreCreateMutex();
  sensor2Mutex = xSemaphoreCreateMutex();

  gsrMutex = xSemaphoreCreateMutex();
  hrMutex = xSemaphoreCreateMutex();
  usernameMutex = xSemaphoreCreateMutex();

  xSemaphoreGive(i2cMutex);
  xSemaphoreGive(sensor1Mutex);
  xSemaphoreGive(sensor2Mutex);
  xSemaphoreGive(gsrMutex);
  xSemaphoreGive(hrMutex);
  xSemaphoreGive(usernameMutex);

  // xTaskCreate(servoTask, "servoTask", 2048, NULL, 0, NULL); 
  xTaskCreate(wifiTask, "wifiTask", 2048, NULL, 0, NULL); 
  xTaskCreate(lcdTask, "LCDTask", 2048, NULL, 0, NULL); 
  xTaskCreate(ledTask, "LEDTask", 2048, NULL, 0, NULL); 
}

void servoTask(void* pvParameters) {
  (void) pvParameters;
  long distance = sensor2 - sensor1;
  if(username != "unknown" && (distance >= -3 && distance <= 3)) {
    moveServo(90);
  } else {
    moveServo(0);
  }
}

void wifiTask(void* pvParameters) {
  // Connect to WiFi

  (void) pvParameters;
  client.setClient(espClient);
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(callback);

  Serial.println(WIFI_SSID);
  for (;;) {
    while (WiFi.status() != WL_CONNECTED) {
      Serial.println("Connecting to WiFi...");
      Serial.println(WiFi.waitForConnectResult());
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      vTaskDelay(500);
    }
    // Connect to MQTT broker
    while (!client.connected()) {
      if (client.connect("ESP32", MQTT_USER, MQTT_PASS)) {
        Serial.println("Connected to MQTT broker");
        client.subscribe("/eps-L1/#");
      } else {
        Serial.println("Failed to connect to MQTT broker. Retrying in 5 seconds...");
        vTaskDelay(5000);
      }
    }
      // Handle MQTT messages or other WiFi-related tasks here
      client.loop();
      vTaskDelay(100);  // Adjust as needed
    // }
  }
   
  
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Message arrived on topic: " + String(topic));
  if (strcmp(topic, "/eps-L1/location/sensor1") == 0) {
    String message;
    for(int i=0; i<length; i++) {
      message += (char)payload[i];
    }

    if (xSemaphoreTake(sensor1Mutex, portMAX_DELAY)) {
      sensor1 = message.toInt();
      xSemaphoreGive(sensor1Mutex);
    }

    Serial.print("Sensor 1 value: ");
    Serial.println(message);
  } 

  if (strcmp(topic, "/eps-L1/location/sensor2") == 0) {
    String message;
    for(int i=0; i<length; i++) {
      message += (char)payload[i];
    }

    if (xSemaphoreTake(sensor2Mutex, portMAX_DELAY)) {
      sensor2 = message.toInt();
      xSemaphoreGive(sensor2Mutex);
    }
    
    Serial.print("Sensor 2 value: ");
    Serial.println(message);
  }
  if (strcmp(topic, "/eps-L1/user/HR") == 0) {
    String message;
    for(int i=0; i<length; i++) {
      message += (char)payload[i];
    }

    if (xSemaphoreTake(hrMutex, portMAX_DELAY)) {
      hr = message;
      xSemaphoreGive(hrMutex);
    }
    
    Serial.print("HR value: ");
    Serial.println(message);
  }
   if (strcmp(topic, "/eps-L1/user/GSR") == 0) {
    String message;
    for(int i=0; i<length; i++) {
      message += (char)payload[i];
    }

    if (xSemaphoreTake(gsrMutex, portMAX_DELAY)) {
      gsr = message;
      xSemaphoreGive(gsrMutex);
    }

    Serial.print("GSR value: ");
    Serial.println(message);
  }
   if (strcmp(topic, "/eps-L1/user/prediction") == 0) {
    String message;
    for(int i=0; i<length; i++) {
      message += (char)payload[i];
    }

    if (xSemaphoreTake(gsrMutex, portMAX_DELAY)) {
      username = message;
      xSemaphoreGive(gsrMutex);
    }

    Serial.print("username value: ");
    Serial.println(message);
  } 
  
}

void ledTask(void* pvParameters) {

  (void) pvParameters;

  for(;;) {

  long distance;
  if (xSemaphoreTake(sensor1Mutex, portMAX_DELAY)) {
    if (xSemaphoreTake(sensor2Mutex, portMAX_DELAY)) {
      distance = sensor2 - sensor1;
      xSemaphoreGive(sensor2);
    }
    xSemaphoreGive(sensor1);
  }

    Serial.println("distance: ");
    Serial.println(distance);
    if(distance < -10) {
      ledFarLeft();
      vTaskDelay(500);
    }  else if(distance >= -10 && distance <= -4) { 
      ledCloseLeft();
      vTaskDelay(500);
    } else if(distance >= -3 && distance <= 3) {
      ledCenter();
      vTaskDelay(500);
    } else if(distance >= 4 && distance <= 10) { 
      ledCloseRight();
      vTaskDelay(500);
    } else {
      ledFarRight();
      vTaskDelay(500);
    }
  }
}

void ledFarLeft() {

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x00);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0xC0);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados
    vTaskDelay(1000);

    xSemaphoreGive(i2cMutex);
  }  
}

void ledCloseLeft() {

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x00);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0x30);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados
    vTaskDelay(1000);

    xSemaphoreGive(i2cMutex);
  }  
}

void ledCloseRight() {

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x06);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0x00);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados
    vTaskDelay(1000);

    xSemaphoreGive(i2cMutex);
  }  
}

void ledFarRight() {

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x18);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0x00);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados
    vTaskDelay(1000);

    xSemaphoreGive(i2cMutex);
  }  
}

void ledCenter() {
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x01);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0x08);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados
    vTaskDelay(1000);
    xSemaphoreGive(i2cMutex);
  }  
}

void lcdTask(void* pvParameters) {

  (void) pvParameters;
  lcd.init();
  lcd.backlight();
  miServo.attach(pinServo);
  miServo.write(0);
  for(;;) {

    String currentUsername;
    String currentHr;
    String currentGsr;
    if (xSemaphoreTake(usernameMutex, portMAX_DELAY)) {
      if (xSemaphoreTake(hrMutex, portMAX_DELAY)) {
        if (xSemaphoreTake(gsrMutex, portMAX_DELAY)) {
          currentUsername = username;
          currentHr = hr;
          currentGsr = gsr;
          xSemaphoreGive(gsrMutex);
        }
        xSemaphoreGive(hrMutex);
      }
      xSemaphoreGive(usernameMutex);
    }

    printLCD(currentUsername, currentHr, currentGsr);
  }
}

void printLCD(String username, String hr, String gsr) {
    lcd.setCursor(0,0);
    lcd.print("HR: " + hr + " GSR: " + gsr);
    if(username == "unknown") {
      lcd.setCursor(0,1);
      lcd.print("E: User unknown!");
      moveServo(0);
      vTaskDelay(1000);
      lcd.clear();
    } else {
      lcd.setCursor(0,1);
      lcd.print(username + " accepted");
      long distance;
      if (xSemaphoreTake(sensor1Mutex, portMAX_DELAY)) {
        if (xSemaphoreTake(sensor2Mutex, portMAX_DELAY)) {
          distance = sensor2 - sensor1;
          xSemaphoreGive(sensor2);
        }
        xSemaphoreGive(sensor1);
      }
      if((distance >= -3 && distance <= 3)) {
        moveServo(90);
      }
      vTaskDelay(1000);
      lcd.clear();
    }
    vTaskDelay(1000);
}

void moveServo(int degrees) {

  miServo.write(degrees);

}

void loop() {
}