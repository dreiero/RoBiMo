/*
 * Sensorknotenv3.ino - program to control the project ROBiMo sensor nodes
 * last tested: 11/2022
 * written for Adafruit ItsyBitsy 5V by Otto Dreier (@dreiero on GitHub)
 * 
 * veraion 1.0 - initial program:
 *                                simple sensors (no pH)
 *                                PCB using Adafruit Trinket Pro 5V
 * version 2.0 - second version for the second PCB:
 *                                design still using the Trinket Pro
 *                                added pH sensing
 *                                no longer measuring turbidity
 *                                much smaller footprint
 * version 3.0 - third version for the third PCB:
 *                                Adafruit ItsyBitsy 5V instead of Trinket Pro 5V
 *                                PCB even smaller than previous version
 *                                added USB-Serial ("Serial"), different from RS485-Serial ("Serial1")
 *                                added debug mode
 *                                added query mode
 *                                added calibration for pH
 *                                added calibration for EC
 */
#include <stdio.h>
#include <Wire.h>                   //enables I2C for pH, EC and Orientation Chip Communication
#include <Adafruit_Sensor.h>        
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <string.h>
#include "RoBiMo.h"
  
/*globale Definitionen*/
namespace constants
{
  extern const int DEBUG = 0;               //1 = DEBUG mode on | 0 = DEBUG mode off | sends additional info through the USB Serial ("Serial"). 
  extern const float PROGRAMVERS = 3.0;     //description of program version
  extern const char KnotenID[] = "K1";      //ID as a beginning for the 
  extern const int TIMEOUT = 100;           //Timeout for the RS485 Serial ("Serial1"). default from the library is 1000, changed to 100 for faster responses
  extern const int TEMPCALIB = 0;           //Temperature calibration factor, gets added as the last step in temperature measurement
  extern const int PRESSCALIB = 0;          //Pressure calibration factor, gets added as the last step in Pressure measurement
  extern const float KVALUE = 0.44;         //K value of the electrical conductivity electrodes (for IST-AG electrodes use 0.44)
}
int QUERYMODE = 1;                          //1 = waiting for Master | 0 = sending all the time | decides whether the node should push his new measurements as fast as possible or wait for a query asking him to send

//--------------------------------------begin setup-----------------
void setup()
{
  if(constants::DEBUG == 1)                                  //DEBUG only
  {Serial.begin(9600);}                           //start USB-Serial connection
  Wire.begin();
  Serial1.begin(9600);
  Serial1.setTimeout(constants::TIMEOUT);
  while(!Serial1)
  {while(1);}
  pinMode (ENABLE_PIN, OUTPUT);                   // driver output enable
  pinMode (LED_PIN, OUTPUT);
  pinMode (4,OUTPUT);// LED output enable
  //**************************************
  /* Initialise the sensor */
  bno.begin();
//  if(!bno.begin())
//  {
//    while(1);
//  }
  delay(1000);
  bno.setExtCrystalUse(true);
  set_probe();
  digitalWrite(13, HIGH);
  digitalWrite(4, HIGH);
  delay(1000);
//**************************************
//  byte led_reg = 0x05;                            //LED register; default: blinking 
//  Wire.beginTransmission(C_bus_address);
//  Wire.write(led_reg);
//  Wire.write(0x01);                               //0x01 for blinking each measurement | 0x00 for power saving mode, LED are always off
//  Wire.endTransmission();
//**************************************
  byte active_reg = 0x06;                         //Hibernation register
  Wire.beginTransmission(C_bus_address);
  Wire.write(active_reg);
  Wire.write(0x01);                               //0x01 for waking | 0x00 for hibernation
  Wire.endTransmission();
  Wire.beginTransmission(pH_bus_address);
  Wire.write(active_reg);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(1000);
if (constants::DEBUG == 1)
{
  Serial.println("Debug mode started");
  Serial.print("Program version ");
  Serial.println(constants::PROGRAMVERS);
}

}
//--------------------------------------end setup-------------------

//--------------------------------------begin loop------------------
void loop()
{
  String answer;
  char strTemp[10];
  char strPress[10];
  char strpH[7];
  float pH=0;
  float C=0;
  char strC[10];
  char OrientationX[10];
  char OrientationY[10];
  char OrientationZ[10];
  byte reading_readyC = 0;
  byte reading_readypH = 0;
  int go = 0;
  
  digitalWrite(13,LOW);
  if (QUERYMODE == 1)
  {
    if (Serial1.available() > 0)
    {
      answer = Serial1.readString();
      answer.trim();
      if (constants::DEBUG == 1)
      {
        Serial.println(answer);
      }
      if (answer == constants::KnotenID)
      {
        go = 1;
      }
      else if (answer.indexOf("fastMeasure") != -1)
      {
        if (answer.indexOf(constants::KnotenID) == 0)
        {
          QUERYMODE = 0;
          go = 1;
        }
        else
        {
          QUERYMODE = 1;
          go = 0;
        }
      }
      else if (answer.indexOf("pHCalib") != -1)
      {
        if (answer.indexOf(constants::KnotenID) == 0)
        {
          if (constants::DEBUG == 1)
          {
            Serial.println("pH calibration started");
          }
          pHcalibration();
        }
      }
      else if (answer.indexOf("CCalib") != -1)
      {
        if (answer.indexOf(constants::KnotenID) == 0)
        {
          if (constants::DEBUG == 1)
          {
            Serial.println("EC calibration started");
          }
          Ccalibration();
        }
      }
      else
      {
        go = 0;
      }
    }
  }
  else                            //not in query mode ==> fast measurement
  {
    go = 0;
    if (Serial1.available() > 0)  //if the node can read any message on the RS485 bus ==> switch to query mode
    {
      QUERYMODE = 1;
    }
    else                          //if no message on bus ==> send your own
    {
      go = 1;
    }
  }
  
 if (go == 1)                     //regardless of measuring mode, if go == 1 then send some data!
  {
    digitalWrite(4,LOW);
    /* Get a new orientation sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
    /*read analog sensors and data from orientation sensor*/
    dtostrf(SensorTempRead(), 4, 2, strTemp);
    dtostrf(SensorPressRead(), 4, 3, strPress);
    dtostrf(event.orientation.x, 4, 1, OrientationX);
    dtostrf(event.orientation.y, 4, 1, OrientationY);
    dtostrf(event.orientation.z, 4, 1, OrientationZ);
    /*see if new readings are availeable*/
    reading_readyC = request_reading(C_bus_address);
    reading_readypH = request_reading(pH_bus_address);
    /*send current temp to pH and EC Chip for temp-compensation*/
    temp_comp(strTemp);
    /*read data from pH and EC chips*/
    if (reading_readyC == 1)
    {
      C = SensorCRead();
      check_reading(C_bus_address, reading_readyC);
    }
    if (reading_readypH == 1)
    {
      pH = SensorpHRead();
      check_reading(pH_bus_address, reading_readypH);
    }
    if(constants::DEBUG == 1)                  //DEBUG mode only! ==> sending via USB
    {
      Serial.print(constants::KnotenID);
      Serial.print(",T,");                             
      Serial.print(strTemp);
      Serial.print(",pH,");
      Serial.print(pH, 4);
      Serial.print(",EC,");     
      Serial.print(C, 4);
      Serial.print(",A,"); 
      Serial.print(OrientationX);
      Serial.print(",");
      Serial.print(OrientationY);
      Serial.print(",");
      Serial.print(OrientationZ);
      Serial.print(",p,");
      Serial.print(strPress);
      Serial.print("\r\n");
    }
    delay(100);
    /*Senden der Daten mit ID*/
    digitalWrite (ENABLE_PIN, HIGH);  //enable Communication over RS485 (sending)
    Serial1.print(constants::KnotenID);
    Serial1.print(",T,");                             
    Serial1.print(strTemp);
    Serial1.print(",pH,");
    Serial1.print(pH, 4);
    Serial1.print(",EC,");     
    Serial1.print(C, 4);
    Serial1.print(",A,"); 
    Serial1.print(OrientationX);
    Serial1.print(",");
    Serial1.print(OrientationY);
    Serial1.print(",");
    Serial1.print(OrientationZ);
    Serial1.print(",p,");
    Serial1.print(strPress);
    Serial1.print("\r\n");
    delay(100);
    digitalWrite (ENABLE_PIN, LOW);   //disable RS485 COM
    digitalWrite(4,HIGH);
  }           //end of "if go == 1" ==> end of data to send
  delay(100); //animate the red LED (fast blink), once per loop-function run
  digitalWrite(13,HIGH);
  delay(100);  
}
//--------------------------------------end loop--------------------
