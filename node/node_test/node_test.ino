#include <Wire.h>

void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
  
  pinMode(7, OUTPUT);
  Serial.print("BOT\n");
  Serial.flush();
  delay(2000);
  Serial.print("GOO\n");
  Serial.flush();
  
}

void loop()
{
  digitalWrite(7, HIGH); 
 
  Wire.beginTransmission(0x50); // transmit to EEPROM
  Wire.write(byte(0x00));       // ask for memory dump 
  Wire.endTransmission();       // stop transmitting

  Wire.requestFrom(0x50, 255);  // prepare for memory dump
  
  while(Wire.available())    // slave may send less than requested
  { 
    char c = Wire.read();    // receive a byte as character
    Serial.print(c);         // print the character
  }
  
  digitalWrite(7, LOW); 
  Serial.print("DNE\n");
  Serial.flush();
  delay(500);
}

