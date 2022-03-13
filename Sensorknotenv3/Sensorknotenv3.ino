#include <SoftwareSerial.h>         //enables multiple Serials, esp. for RS485
#include <stdio.h>
#include <Wire.h>                   //enables I2C for pH, EC and Orientation Chip Communication
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
/*globale Definitionen*/
char KnotenID = "K1";

/*Temperature - TEMP*/
#define SERIESRESISTOR 10000        //used reference resistor
#define THERMISTORPIN A0            //analog input pin for temperature data
#define TEMPERATURENOMINAL 25       //temperature for nominal thermistor value
#define TEMPNUMSAMPLES 10           //number of averaged 
#define BCOEFFICIENT 3950           //coefficient for sensor calibration
#define THERMISTORNOMINAL 10000     //normal resistance for nominal thermistor temperature
#define TEMPCALIB 0                 //calibration factor, gets added as the last step in temperature measurement
int TempSamples[TEMPNUMSAMPLES];    //array for temperature measurement samples

/*Pressure - P*/
#define PRESSPIN A1                 //analog input pin for pressure measurement
#define PRESSNUMSAMPLES 10          //number of averaged samples

/*Conductivity - C*/
#define Ci2c_id 0x64                     //default I2C address
byte C_bus_address = Ci2c_id;              //holds the I2C address.

/*Acceleration - A*/
Adafruit_BNO055 bno = Adafruit_BNO055(55);

/*pH values - pH*/
#define pHi2c_id 0x65                     //default I2C address  
byte pH_bus_address = pHi2c_id;              //holds the I2C address

/*Communication - COM*/
const byte ENABLE_PIN = 3;          //enable rs485-chip
//const byte LED_PIN = 13;            //defines status LED pin
SoftwareSerial rs485 (0, 1);        //receive pin, transmit pin

/*Common I2C tasks*/
#define one_byte_read 0x01              //used in a function to read data from the device  
#define two_byte_read 0x02              //used in a function to read data from the device
#define four_byte_read 0x04             //used in a function to read data from the device

char computerdata[20];                  //we make an 20 byte character array to hold incoming data from the serial monitor
char *cmd;                              //char pointer used in string parsing
char *data_byte_1;                      //char pointer used in string parsing
char *data_byte_2;                      //char pointer used in string parsing
byte bytes_received_from_computer = 0;  //we need to know how many character bytes have been received.

union sensor_mem_handler                //declare the use of a union data type
{
  byte i2c_data[4];                     //define a 4 byte array in the union
  long answ;                            //define an long in the union
};
union sensor_mem_handler move_data;     //declare that we will refer to the union as "move_data"
byte antwort;
//--------------------------------------begin sensor functions-------

//--------------------------------------begin temperature read-------
float SensorTempRead()                          //function to read and calculate the temperature
  {
  uint8_t i;
  float average;
  float temp;
  average = 0;
  for (i=0; i<TEMPNUMSAMPLES; i++)
  {
    average += analogRead(THERMISTORPIN);
    delay(10);
  }
  average = average/TEMPNUMSAMPLES;
  average = 1023/average -1;
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
    volt += ((float)analogRead(PRESSPIN)/1024)*5;
  }
  volt = volt/PRESSNUMSAMPLES;
  return volt;
}
//--------------------------------------end pressure read-----------

//--------------------------------------begin common EC and pH functions
byte request_reading(byte bus_address)
{
  byte nra = 0x07;
  byte new_reading_availeable;
  Wire.beginTransmission(bus_address);
  Wire.write(nra);
  Wire.endTransmission();
  Wire.requestFrom(bus_address, (byte)1);
  new_reading_availeable = Wire.read();
  Wire.endTransmission(); 
  return new_reading_availeable; 
}

void check_reading(byte bus_address, byte new_reading_availeable)
{
  byte nra = 0x07;
  Wire.beginTransmission(bus_address);
  Wire.write(nra);
  Wire.write(0x00);
  Wire.endTransmission();
  new_reading_availeable = 0;
}

void temp_comp(char compensation[10])
{                      
  const byte pH_temperature_compensation_register = 0x0E;         //register to write pH
  const byte C_temperature_compensation_register = 0x10;            //register to write EC
  int temp;
  temp = atoi(compensation);
  temp *= 100;                                            //multiply by 100 to remove the decimal point 
  move_data.answ = temp;                                  //move the float to an unsigned long 
  i2c_write_long(pH_temperature_compensation_register, move_data.answ, pH_bus_address);  //write the 4 bytes of the long to the compensation register
  i2c_write_long(C_temperature_compensation_register, move_data.answ, C_bus_address);  //write the 4 bytes of the long to the compensation register
}

//************************************
//************************************
void i2c_read(byte reg, byte number_of_bytes_to_read, byte bus_address)
{                                                                  //used to read 1,2,and 4 bytes: i2c_read(starting register,number of bytes to read)    
  byte i;                                                          //counter 
  antwort = 0x00;
  Wire.beginTransmission(bus_address);                              //call the device by its ID number
  Wire.write(reg);                                                 //transmit the register that we will start from
  Wire.endTransmission();                                          //end the I2C data transmission
  Wire.requestFrom(bus_address,(byte) number_of_bytes_to_read);     //call the device and request to read X bytes
  for (i = number_of_bytes_to_read; i > 0; i--)
  { 
    move_data.i2c_data[i-1] = Wire.read();
    //antwort += move_data.i2c_data[i-1];
  }                                                                //with this code we read multiple bytes in reverse
  Wire.endTransmission();                                          //end the I2C data transmission  
}
//************************************
//************************************
void i2c_write_byte(byte reg, byte data, byte bus_adress)
{                                                                  //used to write a single byte to a register: i2c_write_byte(register to write to, byte data) 
  Wire.beginTransmission(bus_adress);                              //call the device by its ID number
  Wire.write(reg);                                                 //transmit the register that we will start from
  Wire.write(data);                                                //write the byte to be written to the register 
  Wire.endTransmission();                                          //end the I2C data transmission
}
//***********************************
//***********************************
void i2c_write_long(byte reg, unsigned long data, byte bus_address) 
{                                                                 //used to write a 4 bytes to a register: i2c_write_long(register to start at,unsigned long data )                     
  int i;                                                          //counter
  Wire.beginTransmission(bus_address);                             //call the device by its ID number
  Wire.write(reg);                                                //transmit the register that we will start from
  for (i = 3; i >= 0; i--)
  {                                                               //with this code we write multiple bytes in reverse
    Wire.write(move_data.i2c_data[i]);
  }
  Wire.endTransmission();                                         //end the I2C data transmission
}

//--------------------------------------end common EC and pH functions

//--------------------------------------begin conductivity read-----
void set_probe() {
                                                                         
  const byte set_probe_type_register = 0x08;                                //register to read
  float k_value = 0.44;                                                     //used to hold the new k value
  //atof(data_byte_1);                                                      //convert the k value entered from a string to a float
  k_value *= 100;                                                           //multiply by 100 to remove the decimal point
  move_data.answ = k_value;                                                 //move the float to an unsigned long
  i2c_write_byte(set_probe_type_register, move_data.i2c_data[1], C_bus_address);       //write the MSB of the k value to register 0x08
  i2c_write_byte(set_probe_type_register + 1, move_data.i2c_data[0], C_bus_address);   //write the LSB of the k value to register 0x09
}
//---------------------------
float SensorCRead()
{                                                                                    
  const byte conductivity_register = 0x18;                                                          //register to read
  float conductivity;                                                                           //used to hold the new conductivity value
  i2c_read(conductivity_register, four_byte_read, C_bus_address);                                                  //I2C_read(OEM register, number of bytes to read)
  conductivity = move_data.answ;     //antwort;                                                               //move the 4 bytes read into a float
  conductivity /= 100;                                                                              //divide by 100 to get the decimal point
  return conductivity;                                                                    //print info from register block
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
  rs485.begin (9600);
  Wire.begin();
  pinMode (ENABLE_PIN, OUTPUT);                   // driver output enable
  pinMode (13, OUTPUT);                           // LED output enable
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  set_probe();
  digitalWrite(13,HIGH);
  delay(1000);
  
//  byte i2c_device_address = 0x65;
//  byte led_reg = 0x05;
//  Wire.beginTransmission(i2c_device_address);
//  Wire.write(led_reg);
//  Wire.write(0x01);
//  Wire.endTransmission();

  byte i2c_device_address1 = 0x64;
  byte i2c_device_address2 = 0x65;
  byte active_reg = 0x06;
  Wire.beginTransmission(i2c_device_address1);
  Wire.write(active_reg);
  Wire.write(0x01);
  Wire.endTransmission();

  Wire.beginTransmission(i2c_device_address2);
  Wire.write(active_reg);
  Wire.write(0x01);
  Wire.endTransmission();

}

//--------------------------------------end setup-------------------

//--------------------------------------begin loop------------------
void loop()
{
  char answer(10);
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

  //if (rs485.available())
  //{
  //  answer = rs485.read();
  //  if (answer == KnotenID)
  //  {
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
      //dtostrf(SensorCRead(), 4, 4, strC);
      //dtostrf(SensorpHRead(), 5, 2, strpH);
      if (reading_readyC == 1)
      {
        C = SensorCRead();
        check_reading(C_bus_address, reading_readyC);
        digitalWrite(13,LOW);
      }
      if (reading_readypH == 1)
      {
        pH = SensorpHRead();
        check_reading(pH_bus_address, reading_readypH);
        digitalWrite(13,LOW);
      }
      
      /*Senden der Daten mit ID*/
      digitalWrite (ENABLE_PIN, HIGH);                //enable Communication over RS485 (sending)
      rs485.write(KnotenID);
      rs485.write(",T,");                             
      rs485.write(strTemp);
      rs485.write(",pH,");
      rs485.print(pH);
      rs485.write(",EC,");     
      rs485.print(C);
      rs485.write(",A,"); 
      rs485.write(OrientationX);
      rs485.write(",");
      rs485.write(OrientationY);
      rs485.write(",");
      rs485.write(OrientationZ);
      rs485.write(",p,");
      rs485.write(strPress);
      rs485.write("\r\n");
      digitalWrite (ENABLE_PIN, LOW);                 //disable RS485 COM
   // }
    delay(1000);
    digitalWrite(13,HIGH);
  //}  
}
//--------------------------------------end loop--------------------
