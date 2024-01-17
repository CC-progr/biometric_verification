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

// #define SENSOR1_TOPIC "/location/sensor1"
// #define SENSOR2_TOPIC "/location/sensor2"

// #define USER_DATA1_TOPIC "/user/data/gsr"
// #define USER_DATA2_TOPIC "/user/data/heartRate"

#define SENSOR1_TOPIC "/eps-L1/location/sensor1"
#define SENSOR2_TOPIC "/eps-L1/location/sensor2"

#define USER_HR_TOPIC "/eps-L1/user/HR"
#define USER_GSR_TOPIC "/eps-L1/user/GSR"
#define USER_PREDICTION_TOPIC "/eps-L1/user/prediction"

// #define WIFI_TASK_PRIORITY 1
// #define LCD_TASK_PRIORITY 2
// #define LED_TASK_PRIORITY 3
// #define ARDUINO_RUNNING_CORE 0

SemaphoreHandle_t i2cSemaphore;

SemaphoreHandle_t sensor1Mutex;
SemaphoreHandle_t sensor2Mutex;
long sensor1;
long sensor2;

long tmp_diff_center = 1;
long tmp_diff_close_right = 5;
long tmp_diff_far_right = 7;
long tmp_diff_close_left = -4;
long tmp_diff_far_left = -9;
// int8_t distance = 0; //sensor1 - sensor2

String tmp_GSR = "291"; 
String tmp_HR = "91";
String tmp_user1 = "unkown";
String tmp_user2 = "ruben";

SemaphoreHandle_t gsrMutex;
SemaphoreHandle_t heartRateMutex;
String gsr = "0";
String hr = "0";
String username = "unknown";

Servo miServo;
const int pinServo = 23;

SemaphoreHandle_t LeftValueMutex;
SemaphoreHandle_t CenterValueMutex;
SemaphoreHandle_t RightValueMutex;
int8_t LeftValue = -10;
int8_t CenterValue = 0;
int8_t RightValue = 10;

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


void setup2(){
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

  lcd.init();
  lcd.backlight();

  miServo.attach(pinServo);
  miServo.write(0);

  // Create the semaphores
  i2cSemaphore = xSemaphoreCreateMutex();
  sensor1Mutex = xSemaphoreCreateMutex();
  sensor2Mutex = xSemaphoreCreateMutex();

  gsrMutex = xSemaphoreCreateMutex();
  heartRateMutex = xSemaphoreCreateMutex();

  xSemaphoreGive(i2cSemaphore);

  // LeftValueMutex = xSemaphoreCreateMutex();
  // CenterValueMutex = xSemaphoreCreateMutex();
  // RightValueMutex = xSemaphoreCreateMutex();

  // xTaskCreate(servoTask, "servoTask", 2048, NULL, 0, NULL); //1024
  xTaskCreate(wifiTask, "wifiTask", 2048, NULL, 0, NULL); //1024
  xTaskCreate(lcdTask, "LCDTask", 2048, NULL, 0, NULL); //1024
  xTaskCreate(ledTask, "LEDTask", 2048, NULL, 0, NULL); //1024
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    if (client.connect("ESP32", MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(5000);
    }
  }
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
        // client.subscribe("/location/sensor1");
        // client.subscribe("/location/sensor2");
        // client.subscribe("/location/+");
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
  // if (strcmp(topic, "/location/sensor1") == 0) {
    if (strcmp(topic, "/eps-L1/location/sensor1") == 0) {
    // sensor1 = (int8_t)*payload;

    String message;
    for(int i=0; i<length; i++) {
      // Serial.print((char)payload[i]);
      message += (char)payload[i];
    }

    // Serial.print(message);

    sensor1 = message.toInt();
    Serial.print("Sensor 1 value: ");
    Serial.println(sensor1);
  // } else if (strcmp(topic, "/location/sensor2") == 0) {
    } //else 
    if (strcmp(topic, "/eps-L1/location/sensor2") == 0) {

    String message;
    for(int i=0; i<length; i++) {
      // Serial.print((char)payload[i]);
      message += (char)payload[i];
    }

    // Serial.print(message);

    // sensor2 = (int8_t)*payload;
    sensor2 = message.toInt();
    Serial.print("Sensor 2 value: ");
    Serial.println(sensor2);
  } //else 
  if (strcmp(topic, "/eps-L1/user/HR") == 0) {
    String message;
    for(int i=0; i<length; i++) {
      // Serial.print((char)payload[i]);
      message += (char)payload[i];
    }
    hr = message;
    Serial.print("HR value: ");
    Serial.println(hr);
  } //else
   if (strcmp(topic, "/eps-L1/user/GSR") == 0) {
    String message;
    for(int i=0; i<length; i++) {
      // Serial.print((char)payload[i]);
      message += (char)payload[i];
    }
    gsr = message;
    Serial.print("GSR value: ");
    Serial.println(gsr);
  } //else
   if (strcmp(topic, "/eps-L1/user/prediction") == 0) {
    String message;
    for(int i=0; i<length; i++) {
      // Serial.print((char)payload[i]);
      message += (char)payload[i];
    }
    username = message;
    Serial.print("prediction value: ");
    Serial.println(username);
  } //
  
}

void ledTask(void* pvParameters) {

  (void) pvParameters;

  // if (xSemaphoreTake(sensor1Mutex, portMAX_DELAY)) {
  //   if (xSemaphoreTake(sensor2Mutex, portMAX_DELAY)) {
  //     int8_t distance = sensor1 - sensor2;
  //     xSemaphoreGive(sensor2);
  //   }
  //   xSemaphoreGive(sensor1);
  // }
// ---------------------------
  // for(;;) {
  //   if(LeftValue < 0) { // distance
  //     ledLeft();
  //     vTaskDelay(500);
  //   } // else 

  //   if(CenterValue == 0) {
  //     ledCenter();
  //     vTaskDelay(500);
  //   }

  //   if(RightValue > 0) {
  //     ledRight();
  //     vTaskDelay(500);
  //   }
  // }
  // --------------------

  
    for(;;) {
      // centro: [-3, 3]
      // closeLeft: [-9, -4]
      // farLeft: (-inf, -10]
      // closeRight: [4, 9]
      // farRight: [10, inf)
      long distance = sensor2 - sensor1;//sensor1 - sensor2; //tmp_diff_close_right;//sensor1 - sensor2
      Serial.println("distance: ");
      Serial.println(distance);
      // if(distance < -5) { // distance
      if(distance < -10) {
        ledFarLeft();
        vTaskDelay(500);
      }  else if(distance >= -10 && distance <= -4) { // distance
        ledCloseLeft();
        vTaskDelay(500);
      } else if(distance >= -3 && distance <= 3) { // distance
        ledCenter();
        vTaskDelay(500);
      } else if(distance >= 4 && distance <= 10) { // distance
        ledCloseRight();
        vTaskDelay(500);
      } else {// distance > 5
        ledFarRight();
        vTaskDelay(500);
      }
  }
}

void ledFarLeft() {

  // if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x00);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0xC0);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados
    vTaskDelay(1000);

    // xSemaphoreGive(i2cSemaphore);
  // }  
}

void ledCloseLeft() {

  // if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x00);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0x30);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados
    vTaskDelay(1000);

    // xSemaphoreGive(i2cSemaphore);
  // }  
}

void ledCloseRight() {

  // if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x06);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0x00);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados
    vTaskDelay(1000);

    // xSemaphoreGive(i2cSemaphore);
  // }  
}

void ledFarRight() {

  // if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x18);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0x00);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados
    vTaskDelay(1000);

    // xSemaphoreGive(i2cSemaphore);
  // }  
}

void ledLeft() {

  // if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x00);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0xF0);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados
    vTaskDelay(1000);

    // xSemaphoreGive(i2cSemaphore);
  // }  
}

void ledCenter() {
  // if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x01);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0x08);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados
    vTaskDelay(1000);
    // xSemaphoreGive(i2cSemaphore);
  // }  
}

void ledRight(){
  // if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOB);        // B Register
    Wire.write(0x1E);
    Wire.endTransmission();   // Todos los pins a 0 --> todos apagados

    Wire.beginTransmission(MCPADDRESS);
    Wire.write(GPIOA);        // A Register
    Wire.write(0x00);
    Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos
    vTaskDelay(1000);
    // xSemaphoreGive(i2cSemaphore);
  // }  
}

void lcdTask(void* pvParameters) {

  (void) pvParameters;
  //collect user verification data
  // if (xSemaphoreTake(user_data1, portMAX_DELAY)) {
  //   if (xSemaphoreTake(user_data2, portMAX_DELAY)) {
  //     int8_t user_data = userdata1 - userdata2;
  //     xSemaphoreGive(user_data2);
  //   }
  //   xSemaphoreGive(user_data1);
  // }

  // for(;;) {
  //   if(LeftValue < 0) { //user_data
  //     LCDLeft(LeftValue);
  //     vTaskDelay(500);
  //   }

  //   if(CenterValue == 0) {
  //     LCDCenter(CenterValue);
  //     vTaskDelay(500);
  //   }

  //   if(RightValue > 0) {
  //     LCDRight(RightValue);
  //     vTaskDelay(500);
  //   }
  // }
  
  // -----------------
  // semaforos aqui
  for(;;) {
    //TODO las variables las cojo aqui o antes?
    // String username = tmp_user2;
    // String hr = tmp_HR;
    // String gsr = tmp_GSR;

    printLCD(username, hr, gsr);
  }
}

void printLCD(String username, String hr, String gsr) {
    // lcd.setCursor(0,0);
    // lcd.print("HR: ");zx
    // lcd.print(hr);
    // lcd.setCursor(0,1);
    // lcd.print("GSR: ");
    // lcd.print(gsr);
    lcd.setCursor(0,0);
    lcd.print("HR: " + hr + " GSR: " + gsr);
    // vTaskDelay(1000);
    // lcd.clear();
    if(username == "unknown") {
      // lcd.setCursor(0,0);
      // lcd.print("E: User");
      lcd.setCursor(0,1);
      // lcd.print("unknown!");
      lcd.print("E: User unknown!");
      moveServo(0);
      vTaskDelay(1000);
      lcd.clear();
    } else {
      // lcd.setCursor(0,0);
      // lcd.print(username);
      lcd.setCursor(0,1);
      // lcd.print("accepted");
      
      lcd.print(username + " accepted");
      long distance = sensor2 - sensor1;
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


void LCDLeft(int8_t LeftValue) {

  // if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
    uint8_t trueValue = -LeftValue;
    lcd.setCursor(0,1);
    lcd.print(trueValue);
    lcd.print(" cm from left");
    vTaskDelay(1000);
    lcd.clear(); 
    // xSemaphoreGive(i2cSemaphore);
  // }  


  // vTaskDelay(500);
}

void LCDCenter(int8_t CenterValue) {
  // if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
    lcd.setCursor(0,1);
    lcd.print(CenterValue);
    lcd.print(" cm from centre");
    vTaskDelay(1000);
    lcd.clear(); 
    // xSemaphoreGive(i2cSemaphore);
  // }  

  // vTaskDelay(500);
}

void LCDRight(int8_t RightValue) {
  // if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
    lcd.setCursor(0,1);
    lcd.print(RightValue);
    lcd.print(" cm from right");
    vTaskDelay(1000);
    lcd.clear(); 
    // xSemaphoreGive(i2cSemaphore);
  // }  
  // vTaskDelay(500);
}

void setup()
{
  setup2();

  // setupSec();
  
}

void loop()
{

  loop2();
  // loopSec();
  
}

void loop2() {
  //empty
}
// ---------------------
void setupSec() {
  Serial.begin(115200); //9600bps
  Wire.setClock(WIRE_SPEED); // I2C Bus speed
  Wire.begin();     // wake up I2C bus
  
  // MCP Configuration
  Wire.beginTransmission(MCPADDRESS);
  Wire.write(IOCON);  // Sequential access - better performance
  Wire.write(SEQOP | MIRROR | HAEN );
  Wire.endTransmission(); 
 
  Wire.beginTransmission(MCPADDRESS);
  Wire.write(IODIRA);   
  Wire.write(0x00);    // A Register INPUT/ -> OUTPUT
  Wire.write(0x00);    // B Register OUTPUT
  Wire.endTransmission(); 
  
  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPPUA);  // PULL-UP resistors
  Wire.write(0x00);   // A Register
  Wire.write(0x00);   // B Register
  Wire.endTransmission(); 
  lcd.init();
  
  lcd.backlight();
  lcd.print("hello");
  setWifi();
}

void setWifi() {
  Serial.println("Connecting to WiFi ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500);
    Serial.println("Failed to connect...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT broker
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(callback);

  // Serial.println("Connecting to MQTT broker...");
  // while (!client.connected()) {
  //   if (client.connect("ESP32Client")) {
  //     Serial.println("Connected to MQTT broker");
  //     // client.subscribe("/location/sensor1");
  //     // client.subscribe("/location/sensor2");
  //     client.subscribe("/location/+");
  //   } else {
  //     Serial.println("Failed to connect to MQTT broker. Retrying in 5 seconds...");
  //     vTaskDelay(5000);
  //     reconnect();
  //   }
  // }

  // for (;;) {
  //   // Handle MQTT messages or other WiFi-related tasks here
  //   client.loop();
  //   vTaskDelay(100);  // Adjust as needed
  // }


}

void loopSec() {
  int8_t value;
  int8_t valueLeft = -10;
  int8_t valueCenter = 0;
  int8_t valueRight = 10;
  lcd.clear(); 

  if(valueLeft < 0) {
    ledLeft();
    LCDLeft(valueLeft);
    delay(500);
  }

  if(valueCenter == 0) {
    ledCenter();
    LCDCenter(valueCenter);
    delay(500);
  }

  if(valueRight > 0) {
    ledRight();
    LCDRight(valueRight);
    delay(500);
  }
}
 
void center(uint8_t valueCenter) {
  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);        // B Register
  Wire.write(0x01);
  Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);        // A Register
  Wire.write(0x08);
  Wire.endTransmission();   // Todos los pins a 0 --> todos apagados

  lcd.setCursor(0,1);
  lcd.print(valueCenter);
  lcd.print(" cm from centre");
  vTaskDelay(1000);
  lcd.clear(); 

  vTaskDelay(500);
}

void left(int8_t valueLeft) {
  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);        // B Register
  Wire.write(0x00);
  Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);        // A Register
  Wire.write(0xF0);
  Wire.endTransmission();   // Todos los pins a 0 --> todos apagados

  uint8_t trueValue = -valueLeft;

  lcd.setCursor(0,1);
  lcd.print(trueValue);
  lcd.print(" cm from left");
  vTaskDelay(1000);
  lcd.clear(); 

  vTaskDelay(500);
}

void right(uint8_t valueRight) {
  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);        // B Register
  Wire.write(0x1E);
  Wire.endTransmission();   // Todos los pins a 0 --> todos apagados

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);        // A Register
  Wire.write(0x00);
  Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

  lcd.setCursor(0,1);
  lcd.print(valueRight);
  lcd.print(" cm from right");
  vTaskDelay(1000);
  lcd.clear(); 

  vTaskDelay(500);
}
 