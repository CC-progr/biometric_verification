#include "Wire.h"
#include <LiquidCrystal_I2C.h>
 
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

uint16_t time1, time2;
LiquidCrystal_I2C lcd(0x27,16,2);

void setup()
{
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
  
  //Encender la luz de fondo.
  lcd.backlight();
  
}
 
void loop()
{
  uint8_t value;
  int8_t valueLeft = -10;
  uint8_t valueCenter = 0;
  uint8_t valueRight = 10;
  time1 = micros();
  time2 = micros();
  
  Serial.print("Read:");
  Serial.print(value);
  Serial.print(" - Time:");
  Serial.println(time2-time1);
  
  delay(500);

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);        // B Register
  Wire.write(0xFF);
  Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);        // A Register
  Wire.write(0xFF);
  Wire.endTransmission();   // Todos los pins a 1 --> todos encendidos
  //rainbow();

  delay(500);

  lcd.setCursor(0, 0);
  // print message
  lcd.print("Hello, World!");
  delay(1000);
  // clears the display to print new message
  lcd.clear();
  // set cursor to first column, second row
  lcd.setCursor(0,1);
  lcd.print(value);
  lcd.print(" Segundos");
  delay(1000);
  lcd.clear(); 

  if(valueLeft < 0) {
    left(valueLeft);
    delay(500);
  }

  if(valueCenter == 0) {
    center(valueCenter);
    delay(500);
  }

  if(valueRight > 0) {
    right(valueRight);
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
  lcd.print(" cm from centr");
  delay(1000);
  lcd.clear(); 

  delay(500);
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
  delay(1000);
  lcd.clear(); 

  delay(500);
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
  delay(1000);
  lcd.clear(); 

  delay(500);
}

void rainbow() {
  uint8_t ledsToLight00 = 5 & 0b00000000;
  uint8_t ledsToLight01 = 5 & 0b00000001;
  uint8_t ledsToLight02 = 5 & 0b00000011;
  uint8_t ledsToLight03 = 5 & 0b00000010;
  uint8_t ledsToLight04 = 5 & 0b00000110;
  uint8_t ledsToLight05 = 5 & 0b00000100;
  uint8_t ledsToLight06 = 5 & 0b00001100;
  uint8_t ledsToLight07 = 5 & 0b00010000;
  uint8_t ledsToLight08 = 5 & 0b00110000;
  uint8_t ledsToLight09 = 5 & 0b01000000;
  uint8_t ledsToLight10 = 5 & 0b11000000;
  uint8_t ledsToLight11 = 5 & 0b10000000;
  uint8_t ledsToLight12 = 5 & 0b00000000;

  uint8_t ledsToLight13 = 5 & 0b11111111;
  uint8_t ledsToLight14 = 5 & 0b00000000;
  uint8_t ledsToLight15 = 5 & 0b11111111;
  uint8_t ledsToLight16 = 5 & 0b00000000;

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);      // B Register
  Wire.write(ledsToLight00);      // value to send - all HIGH value
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);      // B Register
  Wire.write(ledsToLight01);
  Wire.endTransmission(); 


  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);      // B Register
  Wire.write(ledsToLight02);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);      // B Register
  Wire.write(ledsToLight03);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);      // B Register
  Wire.write(ledsToLight04);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);      // B Register
  Wire.write(ledsToLight05);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);      // B Register
  Wire.write(ledsToLight06);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOA);      // B Register
  Wire.write(ledsToLight07);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);      // B Register
  Wire.write(ledsToLight08);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);      // B Register
  Wire.write(ledsToLight09);
  Wire.endTransmission();

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);      // B Register
  Wire.write(ledsToLight10);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);      // B Register
  Wire.write(ledsToLight11);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);      // B Register
  Wire.write(ledsToLight12);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);      // B Register
  Wire.write(ledsToLight13);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);      // B Register
  Wire.write(ledsToLight14);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);      //ç
  Wire.write(ledsToLight15);
  Wire.endTransmission(); 

  Wire.beginTransmission(MCPADDRESS);
  Wire.write(GPIOB);      // B Register
  Wire.write(ledsToLight16);
  Wire.endTransmission(); 

  }

 


