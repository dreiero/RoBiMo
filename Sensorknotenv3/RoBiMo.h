
/*Temperature - TEMP*/
const int SERIESRESISTOR = 10000;       //used reference resistor
#define THERMISTORPIN A0                //analog input pin for temperature data
const int TEMPERATURENOMINAL = 25;      //temperature for nominal thermistor value
const int TEMPNUMSAMPLES = 10;          //number of averaged 
const int BCOEFFICIENT = 3950;          //coefficient for sensor calibration
const int THERMISTORNOMINAL = 10000;    //normal resistance for nominal thermistor temperature
const int TempSamples[TEMPNUMSAMPLES];  //array for temperature measurement samples
//************************************
//************************************
/*Pressure - P*/
#define PRESSPIN A2                     //analog input pin for pressure measurement
#define PRESSNUMSAMPLES 10              //number of averaged samples
//************************************
//************************************
/*Conductivity - C*/
#define Ci2c_id 0x64                    //default I2C address
byte C_bus_address = Ci2c_id;           //holds the I2C address.
//************************************
//************************************
/*Acceleration - A*/
Adafruit_BNO055 bno = Adafruit_BNO055(55);
//************************************
//************************************
/*pH values - pH*/
#define pHi2c_id 0x65                   //default I2C address  
byte pH_bus_address = pHi2c_id;         //holds the I2C address
//************************************
//************************************
/*Communication - COM*/
//const byte ENABLE_PIN = 7;            //enable rs485-chip
#define ENABLE_PIN 7
const byte LED_PIN = 13;                //defines status LED pin
//SoftwareSerial rs485 (0, 1);          //receive pin, transmit pin
//************************************
//************************************
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
//**************************************
//**************************************
//--------------------------------------begin common EC and pH functions

void i2c_read(byte reg, byte number_of_bytes_to_read, byte bus_address)
{                                                                  //used to read 1,2,and 4 bytes: i2c_read(starting register,number of bytes to read)    
  byte i;                                                          //counter 
  antwort = 0x00;
  Wire.beginTransmission(bus_address);                             //call the device by its ID number
  Wire.write(reg);                                                 //transmit the register that we will start from
  Wire.endTransmission();                                          //end the I2C data transmission
  Wire.requestFrom(bus_address,(byte) number_of_bytes_to_read);    //call the device and request to read X bytes
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
  Wire.beginTransmission(bus_address);                            //call the device by its ID number
  Wire.write(reg);                                                //transmit the register that we will start from
  for (i = 3; i >= 0; i--)
  {                                                               //with this code we write multiple bytes in reverse
    Wire.write(move_data.i2c_data[i]);
  }
  Wire.endTransmission();                                         //end the I2C data transmission
}
//************************************
//************************************
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
//************************************
//************************************
void check_reading(byte bus_address, byte new_reading_availeable)
{
  byte nra = 0x07;
  Wire.beginTransmission(bus_address);
  Wire.write(nra);
  Wire.write(0x00);
  Wire.endTransmission();
  new_reading_availeable = 0;
}
//************************************
//************************************
void hybernate(byte bus_address)
{
  byte nra = 0x06;
  Wire.beginTransmission(bus_address);
  Wire.write(nra);
  Wire.write(0x00);
  Wire.endTransmission();
}
//************************************
//************************************
void temp_comp(char compensation[10])
{                      
  const byte pH_temperature_compensation_register = 0x0E;                               //register to write pH
  const byte C_temperature_compensation_register = 0x10;                                //register to write EC
  int temp;
  temp = atoi(compensation);
  temp *= 100;                                                                          //multiply by 100 to remove the decimal point 
  move_data.answ = temp;                                                                //move the float to an unsigned long 
  i2c_write_long(pH_temperature_compensation_register, move_data.answ, pH_bus_address); //write the 4 bytes of the long to the compensation register
  i2c_write_long(C_temperature_compensation_register, move_data.answ, C_bus_address);   //write the 4 bytes of the long to the compensation register
}
//--------------------------------------end common EC and pH functions
