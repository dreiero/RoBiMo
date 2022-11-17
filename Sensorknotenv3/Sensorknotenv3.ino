/*
 * Sensorknotenv3.ino - program to control the ROBiMo sensor nodes
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
 *                                ToDo: added calibration
 */
#include <stdio.h>
#include <Wire.h>                   //enables I2C for pH, EC and Orientation Chip Communication
#include <Adafruit_Sensor.h>        
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <string.h>
#include "RoBiMo.h"
  
/*globale Definitionen*/
#define DEBUG 0                     //1 = DEBUG mode on | 0 = DEBUG mode off | sends additional info through the USB Serial ("Serial"). 
#define QUERYMODE 1                 //1 = waiting for Master | 0 = sending all the time | decides whether the node should push his new measurements as fast as possible or wait for a query asking him to send
#define PROGRAMVERS 3.0             //description of program version
char KnotenID[] = "K1";             //ID as a beginning for the 
#define TEMPCALIB 0                 //Temperature calibration factor, gets added as the last step in temperature measurement
#define PRESSCALIB 0                //Pressure calibration factor, gets added as the last step in Pressure measurement
#define KVALUE 0.44                 //K value of the electrical conductivity electrodes (for IST-AG electrodes use 0.44)
#define TIMEOUT 100                  //Timeout for the RS485 Serial ("Serial1"). default from the library is 1000, changed to 100 for faster responses
//--------------------------------------begin sensor functions-------
//--------------------------------------begin temperature read-------
float SensorTempRead()                          //function to read and calculate the temperature
  {
  uint8_t i;
  float average;
  float temp;
  average = 0;
  for (i=0; i<TEMPNUMSAMPLES; i++)
    {average += analogRead(THERMISTORPIN);
    delay(10);}
  average = average / TEMPNUMSAMPLES;
  average = 1023 / average -1;
  average = SERIESRESISTOR / average;
  temp = average / THERMISTORNOMINAL;          //(R/Ro)
  temp = log(temp);                            //ln(R/Ro)
  temp = temp / BCOEFFICIENT;                  //1/B * ln(R/Ro)
  temp += 1.0 / (TEMPERATURENOMINAL + 273.15); //+ (1/To)
  temp = 1.0 / temp;                           //Invert
  temp -= 273.15;                              //convert to degrees Celsius
  temp += TEMPCALIB;
  return(temp); 
  }
//--------------------------------------end temperature read--------

//--------------------------------------begin pressure read---------
float SensorPressRead()
{
  float volt = 0;
  float pressure;
  for(int i = 0; i < PRESSNUMSAMPLES; i++)
  {
    volt += ((float)analogRead(PRESSPIN) / 1024) * 5;
  }
  volt = volt / PRESSNUMSAMPLES;
  volt += PRESSCALIB;
  return volt;
}
//--------------------------------------end pressure read-----------

//--------------------------------------begin conductivity read-----
void set_probe() {                                                                        
  const byte set_probe_type_register = 0x08;                                            //register to read
  float k_value = KVALUE;                                                                 //used to hold the new k value
  //atof(data_byte_1);                                                                  //convert the k value entered from a string to a float
  k_value *= 100;                                                                       //multiply by 100 to remove the decimal point
  move_data.answ = k_value;                                                             //move the float to an unsigned long
  i2c_write_byte(set_probe_type_register, move_data.i2c_data[1], C_bus_address);        //write the MSB of the k value to register 0x08
  i2c_write_byte(set_probe_type_register + 1, move_data.i2c_data[0], C_bus_address);    //write the LSB of the k value to register 0x09
}
//---------------------------
float SensorCRead()
{                                                                                    
  const byte conductivity_register = 0x18;                        //register to read
  float conductivity;                                             //used to hold the new conductivity value
  i2c_read(conductivity_register, four_byte_read, C_bus_address); //I2C_read(OEM register, number of bytes to read)
  conductivity = move_data.answ;                                  //answer; move the 4 bytes read into a float
  conductivity /= 100;                                            //divide by 100 to get the decimal point
  return conductivity;                                            //print info from register block
}
//--------------------------------------end conductivity read-------

//--------------------------------------begin pH read---------------
float SensorpHRead()
{
  const byte pH_register = 0x16;                          //register to read
  float pH;                                           //used to hold the new pH value
  i2c_read(pH_register, four_byte_read, pH_bus_address);  //I2C_read(OEM register, number of bytes to read)                  
  pH = move_data.answ;     //antwort;                               //move the 4 bytes read into a float
  pH /= 1000;
  return pH; 
}
//--------------------------------------end pH read-----------------

//--------------------------------------end sensor functions--------

//--------------------------------------begin setup-----------------
void setup()
{
  if(DEBUG == 1)                                  //DEBUG only
  {Serial.begin(9600);}                           //start USB-Serial connection
  Wire.begin();
  Serial1.begin(9600);
  Serial1.setTimeout(TIMEOUT);
  while(!Serial1)
  {while(1);}
  pinMode (ENABLE_PIN, OUTPUT);                   // driver output enable
  pinMode (LED_PIN, OUTPUT);
  pinMode (4,OUTPUT);// LED output enable
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
//  byte i2c_device_address = 0x65;                 //write LED-register for both pH and EC Chip, default: blinking
//  byte led_reg = 0x05;
//  Wire.beginTransmission(i2c_device_address);
//  Wire.write(led_reg);
//  Wire.write(0x01);                               //0x01 for blinking each measurement | 0x00 for power saving mode, LED are always off
//  Wire.endTransmission();
//  byte i2c_device_address1 = 0x64;                //write Hibernation register for EC Chip
//  byte i2c_device_address2 = 0x65;                //write Hibernation register for pH Chip
//  byte active_reg = 0x06;                         //Hibernation register
//  Wire.beginTransmission(i2c_device_address1);
//  Wire.write(active_reg);
//  Wire.write(0x01);                               //0x01 for waking | 0x00 for hibernation
//  Wire.endTransmission();
//  Wire.beginTransmission(i2c_device_address2);
//  Wire.write(active_reg);
//  Wire.write(0x01);
//  Wire.endTransmission();
//  delay(1000);
if (DEBUG == 1)
{
  Serial.println("Debug mode started");
  Serial.print("Program version ");
  Serial.println(PROGRAMVERS);
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
      if (DEBUG == 1)
      {
        Serial.println(answer);
      }
      if (answer == KnotenID)
      {
        go = 1;
      }
      else if (answer == "pHCalib")
      {
        
        go = 0;
      }
      else
      {
        go = 0;
      }
    }
  }
  else
  {
    go = 1;
  }
  if (go == 1)
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
    if(DEBUG == 1)                  //DEBUG mode only!
    {
      Serial.print(KnotenID);
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
    Serial1.print(KnotenID);
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
  }
  delay(100);
  digitalWrite(13,HIGH);
  delay(100);  
}
//--------------------------------------end loop--------------------
