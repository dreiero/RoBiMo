/*
 * RoBiMo.h header for Sensorknoten.ino to control the project RoBiMo sensor nodes 
 * last tested: 21/2022
 * written for adafruit ItsyBitsy 5V by Otto Dreier (@dreiero on GitHub)
 * 
 */


namespace constants                   //here the same namespace, but without values
{
  extern const int DEBUG;             //1 = DEBUG mode on | 0 = DEBUG mode off | sends additional info through the USB Serial ("Serial"). 
  extern const float PROGRAMVERS;     //description of program version
  extern const char KnotenID[];       //ID as a beginning for the 
  extern const int TIMEOUT;           //Timeout for the RS485 Serial ("Serial1"). default from the library is 1000, changed to 100 for faster responses
  extern const int TEMPCALIB;         //Temperature calibration factor, gets added as the last step in temperature measurement
  extern const int PRESSCALIB;        //Pressure calibration factor, gets added as the last step in Pressure measurement
  extern const float KVALUE;          //K value of the electrical conductivity electrodes (for IST-AG electrodes use 0.44)
}

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
byte request_reading(byte bus_address)                            //ask if a new reading is availeable
{
  byte nra = 0x07;                                                //the register that gives this info on both EC and pH chip
  byte new_reading_availeable;                                    //to store our answer
  Wire.beginTransmission(bus_address);                            //begin I2C communication to our given address (either EC or pH)
  Wire.write(nra);                                                //write the wanted register
  Wire.endTransmission();                                         //end transmission ==> chip read our request
  Wire.requestFrom(bus_address, (byte)1);                         //read the I2C bus 
  new_reading_availeable = Wire.read();                           //whatever the answer is, we store it
  Wire.endTransmission();                                         //ending I2C COM for now
  return new_reading_availeable;                                  //return our collected answer
}
//************************************
//************************************
void check_reading(byte bus_address, byte new_reading_availeable) //basically the same as request_reading(), but without getting an answer | used to set the chip as "we have read your data"
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
void temp_comp(char compensation[10])                                                   //to compensate with the current temperature
{                      
  const byte pH_temperature_compensation_register = 0x0E;                               //register to write pH
  const byte C_temperature_compensation_register = 0x10;                                //register to write EC
  int temp;
  temp = atoi(compensation);                                                            //convert the quasi string to integer values
  temp *= 100;                                                                          //multiply by 100 to remove the decimal point 
  move_data.answ = temp;                                                                //move the float to an unsigned long 
  i2c_write_long(pH_temperature_compensation_register, move_data.answ, pH_bus_address); //write the 4 bytes of the long to the compensation register
  i2c_write_long(C_temperature_compensation_register, move_data.answ, C_bus_address);   //write the 4 bytes of the long to the compensation register
}
//************************************
//--------------------------------------end common EC and pH functions

//************************************
//--------------------------------------begin sensor functions-------
//--------------------------------------begin temperature read-------
float SensorTempRead()                          //function to read and calculate the temperature
  {
  uint8_t i;
  float average;
  float temp;
  average = 0;
  for (i=0; i<TEMPNUMSAMPLES; i++)              //collect some data a number of TEMPNUMSAMPLES * times
    {average += analogRead(THERMISTORPIN);      //add up all the data
    delay(10);}
  average = average / TEMPNUMSAMPLES;           //and calculate the average
  average = 1023 / average -1;                  
  average = SERIESRESISTOR / average;
  temp = average / THERMISTORNOMINAL;           //(R/Ro)
  temp = log(temp);                             //ln(R/Ro)
  temp = temp / BCOEFFICIENT;                   //1/B * ln(R/Ro)
  temp += 1.0 / (TEMPERATURENOMINAL + 273.15);  //+ (1/To)
  temp = 1.0 / temp;                            //Invert
  temp -= 273.15;                               //convert to degrees Celsius
  temp += constants::TEMPCALIB;
  return(temp); 
  }
//--------------------------------------end temperature read--------
//************************************
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
  volt += constants::PRESSCALIB;
  return volt;
}
//--------------------------------------end pressure read-----------
//************************************
//--------------------------------------begin conductivity read-----
void set_probe() {                                                                        
  const byte set_probe_type_register = 0x08;                                            //register to read
  float k_value = constants::KVALUE;                                                    //used to hold the new k value
  //atof(data_byte_1);                                                                  //convert the k value entered from a string to a float
  k_value *= 100;                                                                       //multiply by 100 to remove the decimal point
  move_data.answ = k_value;                                                             //move the float to an unsigned long
  i2c_write_byte(set_probe_type_register, move_data.i2c_data[1], C_bus_address);        //write the MSB of the k value to register 0x08
  i2c_write_byte(set_probe_type_register + 1, move_data.i2c_data[0], C_bus_address);    //write the LSB of the k value to register 0x09
}
//************************************
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
//************************************
//--------------------------------------begin pH read---------------
float SensorpHRead()
{
  const byte pH_register = 0x16;                          //register to read
  float pH;                                               //used to hold the new pH value
  i2c_read(pH_register, four_byte_read, pH_bus_address);  //I2C_read(OEM register, number of bytes to read)                  
  pH = move_data.answ;                                    //move the 4 bytes read into a float
  pH /= 1000;
  return pH; 
}
//--------------------------------------end pH read-----------------
//**************************************
//--------------------------------------end sensor functions--------
//**************************************
//--------------------------------------begin pH Calibration--------
//**************************************
void pHcalibration()
{
  const byte calibration_value_register = 0x08;           //register to read / write
  const byte calibration_request_register = 0x0C;         //register to read / write
  const byte calibration_confirmation_register = 0x0D;    //register to read
  const byte cal_clear = 0x01;                            //clear calibration
  const byte cal_low = 0x02;                              //calibrate to a low-point pH value (pH 4)
  const byte cal_mid = 0x03;                              //calibrate to a mid-point pH value (pH 7)
  const byte cal_high = 0x04;                             //calibrate to a high-point pH value(pH 10)
  String answer;                                          //for the RS485 bus messages
  char strTemp[10];                                       //for the temperature values to compensate
  float pH=0;                                             //for the measured pH values
  int middle = 0;                                         //storing whether pH 7 (or any other middle value) was already calibrated or not; has to be calibrated FIRST on each calibration
  byte reading_readypH = 0;

  while (1)
  {
    reading_readypH = request_reading(pH_bus_address);  //see if a new pH value is ready
    dtostrf(SensorTempRead(), 4, 2, strTemp);           //get new temperature value for compensation
    temp_comp(strTemp);                                 //send temperature value as compensation
    if (reading_readypH == 1)                           //if a new reading is available:
    {
      pH = SensorpHRead();                              //get the new pH value
      check_reading(pH_bus_address, reading_readypH);
    }

    digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
    Serial1.println(pH, 4);                             //send current pH values to RS485 bus
    Serial.println(pH, 4);
    delay(500);
    digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
    
    if (Serial1.available() > 0)                        //if we are interrupted by some answer:
    {
      Serial1.readString();                             //read the answer and reset the channel
      if (constants::DEBUG == 1)
      {
        Serial.println("Answer found. Checking which pH value was calibrated");
      }
      while(1)                                          //now we wait for the "real" answer, the actual command
      {
        if (constants::DEBUG == 1)
        {
          Serial.println(".");
        }
        delay(5000);                                    //wait 5 second and then try again
        if (Serial1.available() > 0)                    //if we have a new answer
        {
          answer = Serial1.readString();                //read it
          answer.trim();                                //get rid of any unnecessary stuff like line breaks
          if (constants::DEBUG == 1)
          {
            Serial.println("answer was: ");
            Serial.println(answer);
          }
          if (answer == "7")                            //toggle between different possible answers: this is the starting point
          {
            pH *= 1000;                                                                   //multiply by 100 to remove the decimal point 
            move_data.answ = pH;                                                          //move the float to an unsigned long 
            i2c_write_long(calibration_value_register, move_data.answ, pH_bus_address);
            i2c_write_byte(calibration_request_register, cal_mid, pH_bus_address);        //write the calibration command to the calibration control register  
            delay(165);
            i2c_read(calibration_confirmation_register, one_byte_read, pH_bus_address);   //read from the calibration control register to confirm it is set correctly
            if (bitRead(move_data.i2c_data[0], 1)== 1)
            {
              digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
              Serial1.println("pH 7 calibrated");                 //send current pH values to RS485 bus
              delay(100);
              digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
              if (constants::DEBUG == 1)
              {
                Serial.println("pH 7 calibrated");
              }
              middle = 1;                                         //middlepoint has been set
            }
            break;
          }
          else if (answer == "4")
          {
            if (middle == 1)                                                                //if no middlepoint is set, this value is useless
            {
              pH *= 1000;                                                                   //multiply by 100 to remove the decimal point 
              move_data.answ = pH;                                                          //move the float to an unsigned long 
              i2c_write_long(calibration_value_register, move_data.answ, pH_bus_address);
              i2c_write_byte(calibration_request_register, cal_low, pH_bus_address);        //write the calibration command to the calibration control register  
              delay(165);
              i2c_read(calibration_confirmation_register, one_byte_read, pH_bus_address);   //read from the calibration control register to confirm it is set correctly
              if (bitRead(move_data.i2c_data[0], 0)== 1)
              {
                digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
                Serial1.println("pH 4 calibrated");                 //send current pH values to RS485 bus
                delay(100);
                digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
                if (constants::DEBUG == 1)
                {
                  Serial.println("pH 4 calibrated");
                }
              }
            }
            else
            {
              digitalWrite (ENABLE_PIN, HIGH);
              Serial1.println("pH 4 was not calibrated - middle point missing!");
              delay(100);
              digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
              if (constants::DEBUG == 1)
              {
                Serial.println("pH 4 was not calibrated - middle point missing!");
              }
            }
            break;   
          }
          else if (answer == "10")
          {
            if (middle == 1)                                                                //if no middlepoint is set, this value is useless
            {
              pH *= 1000;                                                                   //multiply by 100 to remove the decimal point 
              move_data.answ = pH;                                                          //move the float to an unsigned long 
              i2c_write_long(calibration_value_register, move_data.answ, pH_bus_address);
              i2c_write_byte(calibration_request_register, cal_high, pH_bus_address);        //write the calibration command to the calibration control register  
              delay(165);
              i2c_read(calibration_confirmation_register, one_byte_read, pH_bus_address);   //read from the calibration control register to confirm it is set correctly
              if (bitRead(move_data.i2c_data[0], 2)== 1)
              {
                digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
                Serial1.println("pH 10 calibrated");                 //send current pH values to RS485 bus
                delay(100);
                digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
                if (constants::DEBUG == 1)
                {
                  Serial.println("pH 10 calibrated");
                }
              }
            }
            else
            {
              digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
              Serial1.println("pH 10 was not calibrated - middle point missing!");
              delay(100);
              digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
              if (constants::DEBUG == 1)
              {
                Serial.println("pH 10 was not calibrated - middle point missing!");
              }
            }
            break;
          }
          else if (answer == "finished")                        //if the user wants to get out of calibration mode
          {
            if (constants::DEBUG == 1)
            {
              Serial.println("calibration finished");
            }
            digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
            Serial1.println("exit calibration");
            delay(100);
            digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
            return(0);
          }
          else
          {
            continue;
          }
        }
      }
    }
  }
}
//**************************************
//--------------------------------------end pH Calibration-----------------------------------
//**************************************
//--------------------------------------begin EC Calibration-----------------------------------
//**************************************
void Ccalibration() {                                               //this function is fairly similar to the calibration of the pH chip. Only the cases to calibrate to are different
  const byte calibration_register = 0x0A;                           //register to read / write
  const byte calibration_request_register = 0x0E;                   //register to read / write
  const byte calibration_confirmation_register = 0x0F;              //register to read
  const byte cal_clear = 0x01;                                      //clear calibration
  const byte cal_dry = 0x02;                                        //dry calibration
  const byte cal_single = 0x03;                                     //single point calibration
  const byte cal_low = 0x04;                                        //calibrate to a low-point EC value
  const byte cal_high = 0x05;                                       //calibrate to a high-point EC value
  float C = 0;                                                      //used to hold the new calibration value
  String answer;                                                    //for the RS485 bus messages
  char strTemp[10];                                                 //for the temperature values to compensate
  byte reading_readyC = 0;

while (1)
  {
    reading_readyC = request_reading(C_bus_address);  //see if a new pH value is ready
    dtostrf(SensorTempRead(), 4, 2, strTemp);           //get new temperature value for compensation
    temp_comp(strTemp);                                 //send temperature value as compensation
    if (reading_readyC == 1)                           //if a new reading is available:
    {
      C = SensorCRead();                              //get the new pH value
      check_reading(C_bus_address, reading_readyC);
    }
    digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
    Serial1.println(C, 4);                             //send current pH values to RS485 bus
    if (constants::DEBUG == 1)
      {
        Serial.println(C, 4);
      }
    delay(500);
    digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
    if (Serial1.available() > 0)
    {
      Serial1.readString();
      if (constants::DEBUG == 1)
      {
        Serial.println("Answer found. Checking how EC value was calibrated");
      }
      while(1)
      {
        if (constants::DEBUG == 1)
        {
          Serial.println(".");
        }
        delay(5000);
        if (Serial1.available() > 0)
        {
          answer = Serial1.readString();
          answer.trim();
          if (constants::DEBUG == 1)
          {
            Serial.println("answer was: ");
            Serial.println(answer);
          }
          if (answer == "low")
          {
            C = 12880;                                                                   //set low-point calibration solution to 12,880 µS. CHANGE IF USING A DIFFERENT SOLUTION!
            C *= 100;                                                                    //multiply by 100 to remove the decimal point 
            move_data.answ = C;                                                          //move the float to an unsigned long 
            i2c_write_long(calibration_register, move_data.answ, C_bus_address);
            i2c_write_byte(calibration_request_register, cal_low, C_bus_address);        //write the calibration command to the calibration control register  
            delay(100);
            i2c_read(calibration_confirmation_register, one_byte_read, C_bus_address);   //read from the calibration control register to confirm it is set correctly
            if (bitRead(move_data.i2c_data[0], 2)== 1)
            {
              digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
              Serial1.println("low-point calibration done");      //send current pH values to RS485 bus
              delay(100);
              digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
              if (constants::DEBUG == 1)
              {
                Serial.println("low-point calibration done");
              }
            }
            break;
          }
          else if (answer == "clear")
          {
            i2c_write_byte(calibration_request_register, cal_clear, C_bus_address);       //write the calibration clear command to the calibration control register
            delay(100);                                                                   //wait for the calibration event to finish
            i2c_read(calibration_confirmation_register, one_byte_read, C_bus_address);    //read from the calibration control register to confirm it is set correctly
            break;
          }
          else if (answer == "high")
          {
            C = 80000;                                                                    //set high-point calibration solution to 80,000 µS. CHANGE IF USING A DIFFERENT SOLUTION!
            C *= 100;                                                                     //multiply by 100 to remove the decimal point 
            move_data.answ = C;                                                           //move the float to an unsigned long 
            i2c_write_long(calibration_register, move_data.answ, C_bus_address);
            i2c_write_byte(calibration_request_register, cal_high, C_bus_address);        //write the calibration command to the calibration control register  
            delay(100);
            i2c_read(calibration_confirmation_register, one_byte_read, C_bus_address);    //read from the calibration control register to confirm it is set correctly
            if (bitRead(move_data.i2c_data[0], 3)== 1)
            {
              digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
              Serial1.println("high-point calibration done");     //send current pH values to RS485 bus
              delay(100);
              digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
              if (constants::DEBUG == 1)
              {
                Serial.println("high-point calibration done");
              }
            }
            break;
          }
          else if (answer == "dry")
          {
            i2c_write_byte(calibration_request_register, cal_dry, C_bus_address);        //write the calibration command to the calibration control register  
            delay(15);
            i2c_read(calibration_confirmation_register, one_byte_read, C_bus_address);   //read from the calibration control register to confirm it is set correctly
            if (bitRead(move_data.i2c_data[0], 0)== 1)
            {
              digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
              Serial1.println("dry calibration done");            //send current pH values to RS485 bus
              delay(100);
              digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
              if (constants::DEBUG == 1)
              {
                Serial.println("dry calibration done");
              }
            }
            break;
          }
          /*else if (answer == "point")
          {
            C *= 100;                                                                    //multiply by 100 to remove the decimal point 
            move_data.answ = C;                                                          //move the float to an unsigned long 
            i2c_write_long(calibration_register, move_data.answ, C_bus_address);
            i2c_write_byte(calibration_request_register, cal_single, C_bus_address);     //write the calibration command to the calibration control register  
            delay(100);
            i2c_read(calibration_confirmation_register, one_byte_read, C_bus_address);   //read from the calibration control register to confirm it is set correctly
            if (bitRead(move_data.i2c_data[0], 1)== 1)
            {
              digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
              Serial1.println("single point calibration done");   //send current pH values to RS485 bus
              delay(100);
              digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
              if (constants::DEBUG == 1)
              {
                Serial.println("single point calibration done");
              }
            }
            break;
          }*/
          else if (answer == "finished")
          {
            if (constants::DEBUG == 1)
            {
              Serial.println("calibration finished");
            }
            digitalWrite (ENABLE_PIN, HIGH);                    //enable RS485 COM
            Serial1.println("calibration finished");
            delay(100);
            digitalWrite (ENABLE_PIN, LOW);                     //disable RS485 COM
            return(0);
          }
          else
          {
            continue;
          }
        }
      }
    }
  } 
}
