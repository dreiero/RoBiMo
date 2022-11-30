/*
 * Sensorknotenv3.ino - program to control the project ROBiMo sensor nodes
 * last tested: 11/2022 on Arduino IDE 1.8.19
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
 *                                ToDo:
 *                                  - change initialisation of orientation sensor, so that he always starts...
 *                                  - add USB serial read so that you can calibrate the chips via USB
 *                                
 */
#include <stdio.h>
#include <Wire.h>                   //enables I2C for pH, EC and Orientation Chip Communication
#include <Adafruit_Sensor.h>        //to get access to the adafruit libraries see here: https://learn.adafruit.com/introducting-itsy-bitsy-32u4/arduino-ide-setup
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <string.h>               //necessary for fancier string functions, not needed right now
#include "RoBiMo.h"
  
/*globale Definitionen*/
namespace constants                         //namespace and extern const to be able to access the constants both in Sensorknotenv3.ino and in RoBiMo.h
{
  extern const int DEBUG = 0;               //1 = DEBUG mode on | 0 = DEBUG mode off | sends additional info through the USB Serial ("Serial").
  extern const float PROGRAMVERS = 3.0;     //description of program version
  extern const char KnotenID[] = "K1";      //ID as a name to identify different nodes in one sensor chain; also the start for the measurement data answer
  extern const int TIMEOUT = 100;           //Timeout for the RS485 Serial ("Serial1"). default from the library is 1000, changed to 100 for faster responses
  extern const int TEMPCALIB = 0;           //Temperature calibration factor, gets added as the last step in temperature measurement, functions only to set a new zero point
  extern const int PRESSCALIB = 0;          //Pressure calibration factor, gets added as the last step in Pressure measurement, functions only to set a new zero point
  extern const float KVALUE = 0.44;         //K value of the electrical conductivity electrodes (for IST-AG electrodes use 0.44), depends on the electrodes' geometry
}
int QUERYMODE = 1;                          //1 = waiting for Master | 0 = sending all the time | decides whether the node should push its new measurements as fast as possible or wait for a query asking it to send

//--------------------------------------begin setup-----------------
void setup() //programm starts here, this function is run once at the start
{
  if(constants::DEBUG == 1)                       //DEBUG only
  {Serial.begin(9600);}                           //start USB-Serial connection
  
  Wire.begin();
  Serial1.begin(9600);
  Serial1.setTimeout(constants::TIMEOUT);
  
  while(!Serial1)
  {while(1);}
  
  pinMode (ENABLE_PIN, OUTPUT);                   //driver output enable
  pinMode (LED_PIN, OUTPUT);                      //red LED output enable ==> blink every loop
  pinMode (4,OUTPUT);                             //green LED output enable ==> blinks with every RS485 sending process
  //**************************************
  /* Initialise the orientation sensor */
  bno.begin();
//  if(!bno.begin())                              //original code, caused long waiting time
//  {
//    while(1);
//  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  set_probe();                                    //set_probe sends the correkt k value (KVALUE) to the conductivity chip, needs to be done only at the start
  digitalWrite(13, HIGH);                         //turn LEDs on
  digitalWrite(4, HIGH);
  delay(1000);
//**************************************
//  byte led_reg = 0x05;                          //blue LED register for the measurement chips; default: blinking | code remains here, so that the mode can be changes in the future
//  Wire.beginTransmission(C_bus_address);
//  Wire.write(led_reg);
//  Wire.write(0x01);                             //0x01 for blinking each measurement, default | 0x00 for power saving mode, LED are always off
//  Wire.endTransmission();
//**************************************
  byte active_reg = 0x06;                         //Hibernation register, waking chips up (to measure) is always a good idea. Otherwise you get no answer and the blue LEDs should not blink | if the setup gets changed (for slower measurements) it might be a good idea to let the measurement chips hibernate inbetween measurements. In the "normal" case of RoBiMo they should always be measuring.
  Wire.beginTransmission(C_bus_address);
  Wire.write(active_reg);
  Wire.write(0x01);                               //0x01 for waking | 0x00 for hibernation
  Wire.endTransmission();
  Wire.beginTransmission(pH_bus_address);
  Wire.write(active_reg);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(1000);
  
if (constants::DEBUG == 1)                //just some code to let the user know what he/she is dealing with
{
  Serial.println("Debug mode started");
  Serial.print("Program version ");
  Serial.println(constants::PROGRAMVERS);
}

}
//--------------------------------------end setup-------------------

//--------------------------------------begin loop------------------
void loop() //this function runs continually, until power is cut or reset button is pressed
{
  String answer;        //for storing the answer send to the PCB (right now only for RS485)
  char strTemp[10];     //for storing the temperature data and sending it via RS485
  char strPress[10];    //for storing the pressure data and sending it via RS485       
  float pH=0;
  float C=0;
  char OrientationX[10];
  char OrientationY[10];
  char OrientationZ[10];
  byte reading_readyC = 0;  //to store the mode of the EC chip: is a new reading availeable or not?
  byte reading_readypH = 0; //to store the mode of the pH chip: is a new reading availeable or not?
  int go = 0;               //stores if an answer should be send via RS485 or not ==> goal: reusing the same answer-sending-code for query and fast mode
  
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
