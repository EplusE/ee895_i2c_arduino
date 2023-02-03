/***************************************************************************/
/* sourcefile for "ee895I2c.h" module */
/***************************************************************************/
/*
Read functions for measurement values of the EE895 Sensor via I2C interface.

Copyright 2022 E+E Elektronik Ges.m.b.H.

Disclaimer:
This application example is non-binding and does not claim to be complete with regard
to configuration and equipment as well as all eventualities. The application example
is intended to provide assistance with the EE895 sensor module design-in and is provided "as is".
You yourself are responsible for the proper operation of the products described.
This application example does not release you from the obligation to handle the product safely
during application, installation, operation and maintenance. By using this application example,
you acknowledge that we cannot be held liable for any damage beyond the liability regulations
described.

We reserve the right to make changes to this application example at any time without notice.
In case of discrepancies between the suggestions in this application example and other E+E
publications, such as catalogues, the content of the other documentation takes precedence.
We assume no liability for the information contained in this document.
*/

// Includes
//-----------------------------------------------------------------------------
#include "ee895I2c.h"
#include <Arduino.h>
#include "Wire.h"


enum Errorcode
{
    OKAY = 0,
    ERR_CKSUM = 1,
    ERR_CHANGE_REG = 2,
    TIME_NOT_IN_SPEC = 3,
    FI_COEF_NOT_IN_SPEC = 4,
};

ee895I2c::ee895I2c(void)
{

}

void ee895I2c::getAllMeasurements(float &temperature, int &co2, float &pressure)
{
  unsigned char i2cResponse[8] = {};
  int i = 0;
  Wire.beginTransmission(0x5E); // set register address 0 an simple I2C interface
  Wire.write(READ_ALL_MEASUREMENTS);
  Wire.endTransmission(true);
  // read data from slave device
  Wire.requestFrom(0x5E, 8, true);
  while (Wire.available())
  {
    i2cResponse[i++] = Wire.read();
  }
  temperature = (float)(i2cResponse[2] * 256 + i2cResponse[3]) / 100;
  co2 = (float)(i2cResponse[0] * 256 + i2cResponse[1]);
  pressure = (float)(i2cResponse[6] * 256 + i2cResponse[7]) / 10;
}

uint8_t ee895I2c::getTempC(float &temperature) 
{
  unsigned char i2cResponse[9] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_TEMPERATURE_CELSIUS, 0x02, 8) == 0)
  {
    temperature = bytesToValue(i2cResponse);
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::getTempF(float &temperature) 
{
  unsigned char i2cResponse[9] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_TEMPERATURE_FAHRENHEIT, 0x02, 8) == 0)
  {
    temperature = bytesToValue(i2cResponse);
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::getTempK(float &temperature) 
{
  unsigned char i2cResponse[9] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_TEMPERATURE_KELVIN, 0x02, 8) == 0)
  {
    temperature = bytesToValue(i2cResponse);
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::getCo2AverWithPc(int &co2) // pressure compensated 
{
  unsigned char i2cResponse[9] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_CO2_AVERAGE_PC, 0x02, 8) == 0)
  {
    co2 = bytesToValue(i2cResponse);
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::getCo2RawWithPc(int &co2) // pressure compensated 
{
  unsigned char i2cResponse[9] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_CO2_RAW_PC, 0x02, 8) == 0)
  {
    co2 = bytesToValue(i2cResponse);
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::getCo2AverWithNpc(int &co2) // not pressure compensated
{
  unsigned char i2cResponse[9] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_CO2_AVERAGE_NPC, 0x02, 8) == 0)
  {
    co2 = bytesToValue(i2cResponse);
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::getCo2RawWithNpc(int &co2) // not pressure compensated
{
  unsigned char i2cResponse[9] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_CO2_RAW_NPC, 0x02, 8) == 0)
  {
    co2 = bytesToValue(i2cResponse);
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::getPressureMbar(float &pressure) // in mbar
{
  unsigned char i2cResponse[9] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_PRESSURE_MBAR, 0x02, 8) == 0)
  {
    pressure = bytesToValue(i2cResponse);
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::getPressurePsi(float &pressure) // in psi
{
  unsigned char i2cResponse[9] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_PRESSURE_PSI, 0x02, 8) == 0)
  {
    pressure = bytesToValue(i2cResponse);
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::readSerialNumber(unsigned char SerialNumber[]) 
{
  unsigned char i2cResponse[22] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_SERIAL_NUMBER, 0x08, 20) == 0)
  {
    for (int i = 2; i < 18; i++)
    {
      SerialNumber[i-2] = i2cResponse[i];
    }
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::readFirmwareVersion(unsigned char FwVersion[]) 
{
  unsigned char i2cResponse[6] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_FIRMWARE_VERSION, 0x01, 6) == 0)
  {
    FwVersion[0] = i2cResponse[2];
    FwVersion[1] = i2cResponse[3];
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::readSensorname(char Sensorname[]) 
{
  unsigned char i2cResponse[22] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_SENSORNAME, 0x08, 20) == 0)
  {
    for (int i = 2; i < 18; i++)
    {
      Sensorname[i-2] = i2cResponse[i];
    }
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::readMeasuringMode(bool &MeasuringMode) // 0 => continuous mode and 1 => single shot 
{
  unsigned char i2cResponse[6] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_MEASURING_MODE, 0x01, 6) == 0)
  {
     MeasuringMode = ((i2cResponse[3] & 0x01));
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::changeMeasuringMode(bool MeasuringMode) // 0 => continuous mode and 1 => single shot 
{
  uint8_t sendByte[2] = {0x00, MeasuringMode};
  return writeToRegister(REGISTER_MEASURING_MODE, sendByte);
}

uint8_t ee895I2c::dataReady(bool &dataAvailable) 
{
  unsigned char i2cResponse[6] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_MEASURING_STATUS, 0x01, 6) == 0)
  {
    dataAvailable = ((i2cResponse[3] & 0x01));
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::triggerReady(bool &triggerAvailable) 
{
  unsigned char i2cResponse[6] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_MEASURING_STATUS, 0x01, 6) == 0)
  {
    triggerAvailable = ((i2cResponse[3] & 0x02));
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::triggerNewMeasurement()
{
  uint8_t sendByte[2] = {0x00, 0x01};
  return writeToRegister(REGISTER_MEASURING_TRIGGER, sendByte);
}

uint8_t ee895I2c::statusDetails(uint8_t &detailedStatus) // Bit 0 = Co2 measurment too high, Bit 1 = Co2 measurment too low, Bit 2 = T measurement too high, Bit 3 = T measurement too low
{                                                       // Bit 6 = p measurement too high, Bit 7 p measurement too low
  unsigned char i2cResponse[6] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_DETAILED_STATUS, 0x01, 6) == 0)
  {
    detailedStatus = i2cResponse[3];
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::changeCo2MeasuringInterval(int measuringInterval) // min = 100 equivalent to 10 s,  max = 36000 equivalent to 1 hour 
{
  if(36001 > measuringInterval && measuringInterval > 99)
  {
    uint8_t sendByte[2] = {(measuringInterval >> 8), (measuringInterval & 0xFF)};
    return writeToRegister(REGISTER_CO2_MEASURING_INTERVAL, sendByte);
  }
  else
  {
    return 3;
  }
}

uint8_t ee895I2c::readCo2MeasuringInterval(int &measuringInterval) // in 0.1 s steps
{
  unsigned char i2cResponse[6] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_CO2_MEASURING_INTERVAL, 0x01, 6) == 0)
  {
    measuringInterval = (i2cResponse[2] << 8) + i2cResponse[3];
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::changeCo2FilterCoefficient(int filterCoefficient) // min = 1 equivalent to 100% step response, max = 20 equivalent to 63% step response
{
  if(21 > filterCoefficient && 0 < filterCoefficient)
  {
    uint8_t sendByte[2] = {0x00, filterCoefficient};
    return writeToRegister(REGISTER_CO2_FILTER_COEFFICIENT, sendByte);
  }
  else
  {
    return 4;
  }
}

uint8_t ee895I2c::readCo2FilterCoefficient(int &filterCoefficient) // min = 1 equivalent to 100% step response, max = 20 equivalent to 63% step response
{
  unsigned char i2cResponse[6] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_CO2_FILTER_COEFFICIENT, 0x01, 6) == 0)
  {
    filterCoefficient = (i2cResponse[2] << 8) + i2cResponse[3];
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::changeCo2CustomOffset(int customOffset) // min = -32786,  max = 32785  
{
  uint8_t sendByte[2] = {(customOffset >> 8), (customOffset & 0xFF)};
  return writeToRegister(REGISTER_CO2_CUSTOMER_OFFSET, sendByte);
}

uint8_t ee895I2c::readCo2CustomOffset(int &customOffset) // min = -32786,  max = 32785  
{
  unsigned char i2cResponse[6] = {};
  if(readBytesFromRegister(i2cResponse, REGISTER_CO2_CUSTOMER_OFFSET, 0x01, 6) == 0)
  {
    customOffset = (i2cResponse[2] << 8) + i2cResponse[3];
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::changeCustomerRegister1(int customerRegister)  // since firmware version 1.1.1
{
  uint8_t sendByte[2] = {(customerRegister >> 8), (customerRegister & 0xFF)};
  return writeToRegister(USER_REGISTER_1, sendByte);
}

uint8_t ee895I2c::readCustomerRegister1(int &customerRegister) // since firmware version 1.1.1
{
  unsigned char i2cResponse[6] = {};
  if(readBytesFromRegister(i2cResponse, USER_REGISTER_1, 0x01, 6) == 0)
  {
    customerRegister = (i2cResponse[2] << 8) + i2cResponse[3];
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::changeCustomerRegister2(int customerRegister) // since firmware version 1.1.1
{
  uint8_t sendByte[2] = {(customerRegister >> 8), (customerRegister & 0xFF)};
  return writeToRegister(USER_REGISTER_2, sendByte);
}

uint8_t ee895I2c::readCustomerRegister2(int &customerRegister) // since firmware version 1.1.1
{
  unsigned char i2cResponse[6] = {};
  if(readBytesFromRegister(i2cResponse, USER_REGISTER_2, 0x01, 6) == 0)
  {
    customerRegister = (i2cResponse[2] << 8) + i2cResponse[3];
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::writeToRegister(uint16_t registerAddress, uint8_t bytesToWrite[])
{
  unsigned char Command[7] = {FUNCTION_CODE_WRITE_REGISTER, (registerAddress >> 8), (registerAddress & 0xFF), bytesToWrite[0], bytesToWrite[1], 0x00, 0x00}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Write, (2 Bytes) CRC CHECKSUM
  uint16_t crc = calcCrc16(Command, 6);
  Command[5] = crc & 0xFF;
  Command[6] = crc >> 8;
  unsigned char i2cResponse[7] = {};
  wireWrite(Command, 6, true);
  wireRead(i2cResponse, 7);
  if(memcmp(Command, i2cResponse, 7) == 0)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::readBytesFromRegister(unsigned char buf[], uint16_t registerAddress, uint16_t registerToRead, uint8_t bytesToRead)
{
  uint16_t crc16_check = 0;
  unsigned char Command[] = {FUNCTION_CODE_READ_REGISTER, (registerAddress >> 8), (registerAddress & 0xFF), (registerToRead >> 8), (registerToRead & 0xFF), 0x00, 0x00}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Register to Read, (2 Bytes) CRC CHECKSUM
  uint16_t crc = calcCrc16(Command, 6);
  Command[5] = crc & 0xFF;
  Command[6] = crc >> 8;
  wireWrite(Command, 6, false);
  wireRead(buf, bytesToRead);
  crc16_check = (buf[(bytesToRead - 1)] << 8) + buf[(bytesToRead - 2)]; 
  if (crc16_check == calcCrc16(buf, (bytesToRead-1)))
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

float ee895I2c::bytesToValue(unsigned char buf[])
{
    uint32_t xh = (buf[4] << 8 | buf[5]);
    uint16_t xl = (buf[2]  << 8 | buf[3]);
    uint32_t x = (xl | xh << 16);
    float value = *(float*)&x;
    return value;
}

void ee895I2c::wireWrite(unsigned char buf[], int to, bool stopmessage)
{
  Wire.beginTransmission(address);
  for (int i = 0; i <= to; i++)
  {
    Wire.write(buf[i]);
  }
  Wire.endTransmission(stopmessage);
}

void ee895I2c::wireRead(unsigned char buf[], uint8_t to)
{
  int i = 0;
  Wire.requestFrom(address, to);
  
  while (Wire.available())
  {
    buf[i++] = Wire.read();
  }
}

uint16_t ee895I2c::calcCrc16(unsigned char buf[], unsigned char len)
{
  uint16_t crc = CRC16_ONEWIRE_START;
  unsigned char i;
  unsigned char j;
  unsigned char crcCheckBuf[22];
  for (i = 0; i < len; i++)
  {
    crcCheckBuf[i+1] = buf[i];
  }
    crcCheckBuf[0] = 0x5F;
  for (i = 0; i < len; i++)
  {
    crc ^= (uint16_t) crcCheckBuf[i]; // XOR byte into least sig. byte of crc

    for (j = 8; j != 0; j--) // Loop over each bit
    {
      if ((crc & 0x0001) != 0) // If the LSB is set
      {
        crc >>= 1; // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else // Else LSB is not set
      {
        crc >>= 1; // Just shift right
      }
    }
  }
  return crc;
}

void ee895I2c::getErrorString(uint8_t Status, char errorString[])
{
  memset(errorString, '\0', sizeof(errorString));
  switch (Status)
  {
  case OKAY:
    strcpy(errorString, "Success");
    break;
  case ERR_CKSUM:
    strcpy(errorString, "Checksum error");
    break;
  case ERR_CHANGE_REG:
    strcpy(errorString, "something went wrong, when changing the register");
    break;
  case TIME_NOT_IN_SPEC:
    strcpy(errorString, "Time intervall is to low or to high");
    break;
      case FI_COEF_NOT_IN_SPEC:
    strcpy(errorString, "Filter Coefficient is to low or to high");
    break;
  default:
    strcpy(errorString, "unknown error");
    break;
  }
}