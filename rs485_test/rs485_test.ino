//#include <SoftwareSerial.h>         //enables multiple Serials, esp. for RS485
#include <stdio.h>
#include <Wire.h>                   //enables I2C for pH, EC and Orientation Chip Communication
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
/*globale Definitionen*/
int KnotenID = "1";

/*Communication - COM*/
//const byte ENABLE_PIN = 7;          //enable rs485-chip
#define ENABLE_PIN 7
const byte LED_PIN = 13;            //defines status LED pin
//SoftwareSerial rs485 (0, 1);        //receive pin, transmit pin

//--------------------------------------begin setup-----------------
void setup()
{
  //rs485.begin (9600);
  Serial1.begin(9600)
  ;while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB
  }
   Wire.begin();
  
  pinMode (ENABLE_PIN, OUTPUT);                   // driver output enable
  pinMode (4, OUTPUT);                           // LED output enable
  pinMode (LED_PIN, OUTPUT);
  digitalWrite(4,HIGH);
  
  byte i2c_device_address1 = 0x64;
  byte i2c_device_address2 = 0x65;
  byte active_reg = 0x05;
  Wire.beginTransmission(i2c_device_address1);
  Wire.write(active_reg);
  Wire.write(0x00);
  Wire.endTransmission();
    Wire.beginTransmission(i2c_device_address2);
  Wire.write(active_reg);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(1000);
}

//--------------------------------------end setup-------------------

//--------------------------------------begin loop------------------
void loop()
{
  //if (rs485.available())
  //{
  //  answer = rs485.read();
  //  if (answer == KnotenID)
  //  {
  digitalWrite(4,LOW);
  digitalWrite(LED_PIN, LOW);
      /*Senden der Daten mit ID*/
      digitalWrite (ENABLE_PIN, HIGH);                //enable Communication over RS485 (sending)
      Serial1.println("hello world");
      //Serial1.println("\r\n");
      delay(1000);
      digitalWrite (ENABLE_PIN, LOW);                 //disable RS485 COM
   // }
    delay(1000);
    digitalWrite(4,HIGH);
    digitalWrite(LED_PIN,HIGH);
    delay(1000);
  //}  
}
//--------------------------------------end loop--------------------
