#include <SoftwareSerial.h>         //enables multiple Serials, esp. for RS485
#include <stdio.h>
#include <Wire.h>                   //enables I2C
#include <Adafruit_FXAS21002C.h>    //enables easy usage of Gyroscope
#include <Adafruit_FXOS8700.h>      //enables easy usage of Accelerometer & Magnetometer
#include <Adafruit_Sensor.h>        //necessary for easy usage of adafruit sensors

/*Temperature - TEMP*/
#define SERIESRESISTOR 10040        //used reference resistor
#define THERMISTORPIN A0            //analog input pin for temperature data
#define TEMPERATURENOMINAL 25       //temperature for nominal thermistor value
#define TEMPNUMSAMPLES 10           //number of averaged 
#define BCOEFFICIENT 3950           //coefficient for sensor calibration
#define THERMISTORNOMINAL 10000     //normal resistance for nominal thermistor temperature
#define TEMPCALIB -4.5              //calibration factor, gets added as the last step in temperature measurement
int TempSamples[TEMPNUMSAMPLES];    //array for temperature measurement samples

/*Pressure - P*/
#define PRESSPIN A2                 //analog input pin for pressure measurement
#define PRESSNUMSAMPLES 10          //number of averaged samples

/*Conductivity - C*/
#define adress 100                  //default I2C ID number for EZO Electrical Conductivity Circuit
byte code = 0;                      //for holding the I2C response
char CData[32];                     //32 byte char array for holfing incoming Data from EC circuit
byte in_char = 0;                   //1 byte buffer to store inbound bytes from EC circuit

/*Turbidity - TURB*/
#define TURBSENSPIN A1              //analog input pin for turbidity measurement
#define TURBNUMSAMPLES 10           //number of averaged measurement samples

/*Acceleration - A*/
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);           //declaration for FXAS21992C gyroscope as Adafruit sensor structure (?)
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);     //declaration for FXOS8700 Accelerometer & Magnetometer as Adafruit sensor structure (?)

/*Communication - COM*/
const byte ENABLE_PIN = 3;          //enable rs485-chip
//const byte LED_PIN = 13;            //defines status LED pin
SoftwareSerial rs485 (0, 1);        //receive pin, transmit pin

//--------------------------------------begin sensor functions-------

//--------------------------------------begin temperature read-------
float SensorTempRead()                          //function to read and calculate the temperature
  {
  uint8_t i;
  float average;
  
  average = 0;
  for (i=0; i<TEMPNUMSAMPLES; i++)
  {
    average += analogRead(THERMISTORPIN);
    delay(10);
  }
  average = average/TEMPNUMSAMPLES;
  average = 1023/average -1;
  average = SERIESRESISTOR / average;
  
  float temp;
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
    volt += ((float)analogRead(PRESSPIN)/1024)*5;
  }
  volt = volt/PRESSNUMSAMPLES;
  return volt;
}
//--------------------------------------end pressure read-----------

//--------------------------------------begin turbidity read--------
float SensorTurbRead()
{
  float volt = 0;
  float turbidity;
  for(int i=0; i<TURBNUMSAMPLES; i++)
  {
    volt+=((float)analogRead(TURBSENSPIN)/1023)*5;
  }
  volt = volt/TURBNUMSAMPLES;
  float multiplier=powf(10.0f, 1);
  volt = roundf(volt*multiplier)/multiplier;
  if(volt<2.5)
  {
    turbidity = 3000;  
  }else
  {
    turbidity = -1120*square(volt)+5742*volt-1756.8;
  }
  return turbidity;
}
//--------------------------------------end turbidity read----------

//--------------------------------------begin conductivity read-----
char SensorCAsk()
{
  //int time_ = 570;                            //changeable delay in depending on the command send to EC circuit
  
  Wire.beginTransmission(adress);
  Wire.write('r');
  Wire.endTransmission();
}

char SensorCRead()
{ 
  int i = 0;                                  //counter for CData array
  //delay(time_);
  Wire.requestFrom(adress, 32, 1);
  code = Wire.read();
  switch (code)
  {
    case 1:
      //success
      break;
    case 2:
      //failed
      break;
    case 254:
      //pending
      break;
    case 255:
      //no data
      break;
  }

  while (Wire.available())
  {
    in_char = Wire.read();
    CData[i] = in_char;
    i += 1;
    if (in_char == 0)
    {
      i = 0;
      break;
    }
  }
  return code;
  //return CData;
}
//--------------------------------------end conductivity read-------
//--------------------------------------end sensor functions--------

//--------------------------------------begin setup-----------------
void setup()
{
  rs485.begin (9600);
  Wire.begin();
  pinMode (ENABLE_PIN, OUTPUT);                   // driver output enable
  //pinMode (LED_PIN, OUTPUT);                      // built-in LED
  //digitalWrite(LED_PIN, HIGH);
  
  if (!gyro.begin())
  {
    /*
    digitalWrite(ENABLE_PIN, HIGH);                //enable Communication over RS485 (sending)
    rs485.write(" gyro not found! "); 
    digitalWrite(ENABLE_PIN, LOW);
    */
    //digitalWrite(LED_PIN, LOW);
    while (1);
  }

  if (!accelmag.begin(ACCEL_RANGE_4G))
  {
    /*
    digitalWrite(ENABLE_PIN, HIGH);                //enable Communication over RS485 (sending)
    rs485.write(" accel/mag not found! "); 
    digitalWrite(ENABLE_PIN, LOW);
    */
    //digitalWrite(LED_PIN, LOW);
    while (1);
  }
}

//--------------------------------------end setup-------------------

//--------------------------------------begin loop------------------
void loop()
{
  char strTurb[10];
  char strTemp[10];
  char strPress[10];
  char strAccelX[10];
  char strAccelY[10];
  char strAccelZ[10];
  char strMagX[10];
  char strMagY[10];
  char strMagZ[10];
  char strGyroX[10];
  char strGyroY[10];
  char strGyroZ[10];
  
  SensorCAsk();
  
  for( int i = 0; i < 50; i++ )
  {
    dtostrf(SensorTurbRead(), 5, 1, strTurb);
    dtostrf(SensorTempRead(), 4, 4, strTemp);
    dtostrf(SensorPressRead(), 4, 4, strPress);
    
    sensors_event_t gevent, aevent, mevent;
    accelmag.getEvent(&aevent, &mevent);
    gyro.getEvent(&gevent);
    dtostrf(aevent.acceleration.x, 3, 1, strAccelX);
    dtostrf(aevent.acceleration.y, 3, 1, strAccelY);
    dtostrf(aevent.acceleration.z, 3, 1, strAccelZ);
    
    dtostrf(mevent.magnetic.x, 3, 1, strMagX);
    dtostrf(mevent.magnetic.y, 3, 1, strMagY);
    dtostrf(mevent.magnetic.z, 3, 1, strMagZ);
    
    dtostrf(gevent.gyro.x, 3, 1, strGyroX);
    dtostrf(gevent.gyro.y, 3, 1, strGyroY);
    dtostrf(gevent.gyro.z, 3, 1, strGyroZ);
    
    delay(10);
    // send to slave  
    digitalWrite (ENABLE_PIN, HIGH);                //enable Communication over RS485 (sending)
    rs485.write("1 ");
    rs485.write(strTurb);                           //writes the Datastring to RS485
    rs485.write(" ");
    
    rs485.write(strTemp);
    rs485.write(" ");

    rs485.write("- "); //dummy-code to keep up the same format?
     
    rs485.write("A ");
    rs485.write(strAccelX);
    rs485.write(" ");
    rs485.write(strAccelY);
    rs485.write(" ");
    rs485.write(strAccelZ);
    
    rs485.write(" M ");
    rs485.write(strMagX);
    rs485.write(" ");
    rs485.write(strMagY);
    rs485.write(" ");
    rs485.write(strMagZ);
    
    rs485.write(" G ");
    rs485.write(strGyroX);
    rs485.write(" ");
    rs485.write(strGyroY);
    rs485.write(" ");
    rs485.write(strGyroZ);
    
    rs485.write(" ");
    rs485.write(strPress);
      
    rs485.write("\r\n");
    digitalWrite (ENABLE_PIN, LOW);                 //disable RS485 COM
  }
  dtostrf(SensorTurbRead(), 5, 1, strTurb);
  dtostrf(SensorTempRead(), 4, 4, strTemp);
  dtostrf(SensorPressRead(), 4, 4, strPress);
  SensorCRead();
  
  sensors_event_t gevent, aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);
  gyro.getEvent(&gevent);
  dtostrf(aevent.acceleration.x, 3, 1, strAccelX);
  dtostrf(aevent.acceleration.y, 3, 1, strAccelY);
  dtostrf(aevent.acceleration.z, 3, 1, strAccelZ);
  
  dtostrf(mevent.magnetic.x, 3, 1, strMagX);
  dtostrf(mevent.magnetic.y, 3, 1, strMagY);
  dtostrf(mevent.magnetic.z, 3, 1, strMagZ);
  
  dtostrf(gevent.gyro.x, 3, 1, strGyroX);
  dtostrf(gevent.gyro.y, 3, 1, strGyroY);
  dtostrf(gevent.gyro.z, 3, 1, strGyroZ);
  
  //delay(5);
  // send to slave  
  digitalWrite (ENABLE_PIN, HIGH);                //enable Communication over RS485 (sending)
  rs485.write("1 ");
  rs485.write(strTurb);                           //writes the Datastring to RS485
  rs485.write(" ");
  
  rs485.write(strTemp);
  rs485.write(" ");
  
  rs485.write(CData);
  rs485.write(" ");
     
  rs485.write("A ");
  rs485.write(strAccelX);
  rs485.write(" ");
  rs485.write(strAccelY);
  rs485.write(" ");
  rs485.write(strAccelZ);
  
  rs485.write(" M ");
  rs485.write(strMagX);
  rs485.write(" ");
  rs485.write(strMagY);
  rs485.write(" ");
  rs485.write(strMagZ);
  
  rs485.write(" G ");
  rs485.write(strGyroX);
  rs485.write(" ");
  rs485.write(strGyroY);
  rs485.write(" ");
  rs485.write(strGyroZ);
  
  rs485.write(" ");
  rs485.write(strPress);
    
  rs485.write("\r\n");
  digitalWrite (ENABLE_PIN, LOW);                 //disable RS485 COM 
}
//--------------------------------------end loop--------------------
