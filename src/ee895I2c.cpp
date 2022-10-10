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
  unsigned char i2cResponse[8];
  int i = 0;
  Wire.beginTransmission(0x5E); // set register address 0 an simple I2C interface
  Wire.write(0);
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
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[9];
  unsigned char Command[] = {0x03, 0x03, 0xEA, 0x00, 0x02, 0xE8, 0xC5}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 8);
  crc16_check = (i2cResponse[7] << 8) + i2cResponse[6]; 
  if (crc16_check == calcCrc16(i2cResponse, 7))
  {
    uint32_t x = (i2cResponse[4] << 24 | i2cResponse[5] << 16 | i2cResponse[2]  << 8 | i2cResponse[3]);
    temperature = *(float*)&x;
    return 0;
  }
  else
  {
    return 1;
  }
}


uint8_t ee895I2c::getTempF(float &temperature) 
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[9];
  unsigned char Command[] = {0x03, 0x03, 0xEC, 0x00, 0x02, 0x08, 0xC4}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 8);
  crc16_check = (i2cResponse[7] << 8) + i2cResponse[6]; 
  if (crc16_check == calcCrc16(i2cResponse, 7))
  {
    uint32_t x = (i2cResponse[4] << 24 | i2cResponse[5] << 16 | i2cResponse[2]  << 8 | i2cResponse[3]);
    temperature = *(float*)&x;
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::getTempK(float &temperature) 
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[9];
  unsigned char Command[] = {0x03, 0x03, 0xF0, 0x00, 0x02, 0xC9, 0x02}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 8);
  crc16_check = (i2cResponse[7] << 8) + i2cResponse[6]; 
  if (crc16_check == calcCrc16(i2cResponse, 7))
  {
    uint32_t x = (i2cResponse[4] << 24 | i2cResponse[5] << 16 | i2cResponse[2]  << 8 | i2cResponse[3]);
    temperature = *(float*)&x;
    return 0;
  }
  else
  {
    return 1;
  }
}


uint8_t ee895I2c::getCo2AverWithPc(int &co2) // pressure compensated 
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[9];
  unsigned char Command[] = {0x03, 0x04, 0x24, 0x00, 0x02, 0x88, 0x4E}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 8);
  crc16_check = (i2cResponse[7] << 8) + i2cResponse[6]; 
  if (crc16_check == calcCrc16(i2cResponse, 7))
  {
    uint32_t x = (i2cResponse[4] << 24 | i2cResponse[5] << 16 | i2cResponse[2]  << 8 | i2cResponse[3]);
    co2 = *(float*)&x;
    return 0;
  }
  else
  {
    return 1;
  }
}


uint8_t ee895I2c::getCo2RawWithPc(int &co2) // pressure compensated 
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[9];
  unsigned char Command[] = {0x03, 0x04, 0x26, 0x00, 0x02, 0x29, 0x8E}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 8);
  crc16_check = (i2cResponse[7] << 8) + i2cResponse[6]; 
  if (crc16_check == calcCrc16(i2cResponse, 7))
  {
    uint32_t x = (i2cResponse[4] << 24 | i2cResponse[5] << 16 | i2cResponse[2]  << 8 | i2cResponse[3]);
    co2 = *(float*)&x;
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::getCo2AverWithNpc(int &co2) // not pressure compensated
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[9];
  unsigned char Command[] = {0x03, 0x04, 0x28, 0x00, 0x02, 0x48, 0x4D}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 8);
  crc16_check = (i2cResponse[7] << 8) + i2cResponse[6]; 
  if (crc16_check == calcCrc16(i2cResponse, 7))
  {
    uint32_t x = (i2cResponse[4] << 24 | i2cResponse[5] << 16 | i2cResponse[2]  << 8 | i2cResponse[3]);
    co2 = *(float*)&x;
    return 0;
  }
  else
  {
    return 1;
  }
}


uint8_t ee895I2c::getCo2RawWithNpc(int &co2) // not pressure compensated
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[9];
  unsigned char Command[] = {0x03, 0x04, 0x2A, 0x00, 0x02, 0xE9, 0x8D}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 8);
  crc16_check = (i2cResponse[7] << 8) + i2cResponse[6];
  if (crc16_check == calcCrc16(i2cResponse, 7))
  {
    uint32_t x = (i2cResponse[4] << 24 | i2cResponse[5] << 16 | i2cResponse[2]  << 8 | i2cResponse[3]);
    co2 = *(float*)&x;
    return 0;
  }
  else
  {
    return 1;
  }
}


uint8_t ee895I2c::getPressureMbar(float &pressure) // in mbar
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[9];
  unsigned char Command[] = {0x03, 0x04, 0xB0, 0x00, 0x02, 0xC9, 0xA2}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 8);
  crc16_check = (i2cResponse[7] << 8) + i2cResponse[6]; 
  if (crc16_check == calcCrc16(i2cResponse, 7))
  {
    uint32_t x = (i2cResponse[4] << 24 | i2cResponse[5] << 16 | i2cResponse[2]  << 8 | i2cResponse[3]);
    pressure = *(float*)&x;
    return 0;
  }
  else
  {
    return 1;
  }
}


uint8_t ee895I2c::getPressurePsi(float &pressure) // in psi
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[9];
  unsigned char Command[] = {0x03, 0x04, 0xB2, 0x00, 0x02, 0x68, 0x62}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 8);
  crc16_check = (i2cResponse[7] << 8) + i2cResponse[6]; 
  if (crc16_check == calcCrc16(i2cResponse, 7))
  {
    uint32_t x = (i2cResponse[4] << 24 | i2cResponse[5] << 16 | i2cResponse[2]  << 8 | i2cResponse[3]);
    pressure = *(float*)&x;
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t ee895I2c::readSerialNumber(unsigned char SerialNumber[]) 
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[22];
  unsigned char Command[] = {0x03, 0x00, 0x00, 0x00, 0x08, 0x49, 0x72}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 20);
  crc16_check = (i2cResponse[19] << 8) + i2cResponse[18]; 
  if (crc16_check == calcCrc16(i2cResponse, 19))
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
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[5];
  unsigned char Command[] = {0x03, 0x00, 0x08, 0x00, 0x01, 0x08, 0xB6}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 6);
  crc16_check = (i2cResponse[5] << 8) + i2cResponse[4]; 
  if (crc16_check == calcCrc16(i2cResponse, 5))
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
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[22];
  unsigned char Command[] = {0x03, 0x00, 0x09, 0x00, 0x08, 0x99, 0x70}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 20);
  crc16_check = (i2cResponse[19] << 8) + i2cResponse[18]; 
  if (crc16_check == calcCrc16(i2cResponse, 19))
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
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[6];
  unsigned char Command[] = {0x03, 0x01, 0xF8, 0x00, 0x01, 0x09, 0x79}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 6);
  crc16_check = (i2cResponse[5] << 8) + i2cResponse[4]; 
  if (crc16_check == calcCrc16(i2cResponse, 5))
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
  unsigned char i2cResponse[7];
  unsigned char sendByte = MeasuringMode;
  unsigned char Command[7] = {0x06, 0x01, 0xF8, 0x00, sendByte, 0x00, 0x00}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Write, (2 Bytes) CRC CHECKSUM
  uint16_t crc = calcCrc16(Command, 6);
  Command[5] = crc & 0xFF;
  Command[6] = crc >> 8;
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


uint8_t ee895I2c::dataReady(bool &dataAvailable) 
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[6];
  unsigned char Command[] = {0x03, 0x01, 0xF9, 0x00, 0x01, 0x58, 0xB9}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 6);
  crc16_check = (i2cResponse[5] << 8) + i2cResponse[4]; 
  if (crc16_check == calcCrc16(i2cResponse, 5))
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
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[6];
  unsigned char Command[] = {0x03, 0x01, 0xF9, 0x00, 0x01, 0x58, 0xB9}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 6);
  crc16_check = (i2cResponse[5] << 8) + i2cResponse[4]; 
  if (crc16_check == calcCrc16(i2cResponse, 5))
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
  unsigned char Command[] = {0x06, 0x01, 0xFA, 0x00, 0x01, 0x64, 0xB9}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Write, (2 Bytes) CRC CHECKSUM
  unsigned char i2cResponse[7];
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

uint8_t ee895I2c::statusDetails(uint8_t &detailedStatus) // Bit 0 = Co2 measurment too high, Bit 1 = Co2 measurment too low, Bit 2 = T measurement too high, Bit 3 = T measurement too low
{                                                      // Bit 6 = p measurement too high, Bit 7 p measurement too low
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[6];
  unsigned char Command[] = {0x03, 0x02, 0x58, 0x00, 0x01, 0x09, 0x1F}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 6);
  crc16_check = (i2cResponse[5] << 8) + i2cResponse[4]; 
  if (crc16_check == calcCrc16(i2cResponse, 5))
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
    unsigned char sendByte0 = measuringInterval / 256;
    unsigned char sendByte1 = measuringInterval % 256;
    unsigned char Command[7] = {0x06, 0x14, 0x50, sendByte0, sendByte1, 0x00, 0x00}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Write, (2 Bytes) CRC CHECKSUM
    uint16_t crc = calcCrc16(Command, 6);
    Command[5] = crc & 0xFF;
    Command[6] = crc >> 8;
    unsigned char i2cResponse[7];
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
  else
  {
    return 3;
  }
}


uint8_t ee895I2c::readCo2MeasuringInterval(int &measuringInterval) // in 0.1 s steps
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[6];
  unsigned char Command[] = {0x03, 0x14, 0x50, 0x00, 0x01, 0x8C, 0x95}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 6);
  crc16_check = (i2cResponse[5] << 8) + i2cResponse[4]; 
  if (crc16_check == calcCrc16(i2cResponse, 5))
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
    unsigned char sendByte = filterCoefficient;
    unsigned char Command[7] = {0x06, 0x14, 0x51, 0x00, sendByte, 0x00, 0x00}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Write, (2 Bytes) CRC CHECKSUM
    uint16_t crc = calcCrc16(Command, 6);
    Command[5] = crc & 0xFF;
    Command[6] = crc >> 8;
    unsigned char i2cResponse[7];
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
  else
  {
    return 4;
  }
}


uint8_t ee895I2c::readCo2FilterCoefficient(int &filterCoefficient) // min = 1 equivalent to 100% step response, max = 20 equivalent to 63% step response
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[6];
  unsigned char Command[] = {0x03, 0x14, 0x51, 0x00, 0x01, 0xDD, 0x55}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 6);
  crc16_check = (i2cResponse[5] << 8) + i2cResponse[4]; 
  if (crc16_check == calcCrc16(i2cResponse, 5))
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
  unsigned char sendByte0 = (customOffset >> 8);
  unsigned char sendByte1 = customOffset & 0xFF;
  unsigned char Command[7] = {0x06, 0x14, 0x52, sendByte0, sendByte1, 0x00, 0x00}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Write, (2 Bytes) CRC CHECKSUM
  uint16_t crc = calcCrc16(Command, 6);
  Command[5] = crc & 0xFF;
  Command[6] = crc >> 8;
  unsigned char i2cResponse[7];
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


uint8_t ee895I2c::readCo2CustomOffset(int &customOffset) // min = -32786,  max = 32785  
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[6];
  unsigned char Command[] = {0x03, 0x14, 0x52, 0x00, 0x01, 0x2D, 0x55}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 6);
  crc16_check = (i2cResponse[5] << 8) + i2cResponse[4]; 
  if (crc16_check == calcCrc16(i2cResponse, 5))
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
  unsigned char sendByte0 = (customerRegister >> 8);
  unsigned char sendByte1 = customerRegister & 0xFF;
  unsigned char Command[7] = {0x06, 0x16, 0xA8, sendByte0, sendByte1, 0x00, 0x00}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Write, (2 Bytes) CRC CHECKSUM
  uint16_t crc = calcCrc16(Command, 6);
  Command[5] = crc & 0xFF;
  Command[6] = crc >> 8;
  unsigned char i2cResponse[7];
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


uint8_t ee895I2c::readCustomerRegister1(int &customerRegister) // since firmware version 1.1.1
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[6];
  unsigned char Command[] = {0x03, 0x16, 0xA8, 0x00, 0x01, 0xC0, 0xDC}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 6);
  crc16_check = (i2cResponse[5] << 8) + i2cResponse[4]; 
  if (crc16_check == calcCrc16(i2cResponse, 5))
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
  unsigned char sendByte0 = (customerRegister >> 8);
  unsigned char sendByte1 = customerRegister & 0xFF;
  unsigned char Command[7] = {0x06,  0x16, 0xA9, sendByte0, sendByte1, 0x00, 0x00}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Write, (2 Bytes) CRC CHECKSUM
  uint16_t crc = calcCrc16(Command, 6);
  Command[5] = crc & 0xFF;
  Command[6] = crc >> 8;
  unsigned char i2cResponse[7];
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


uint8_t ee895I2c::readCustomerRegister2(int &customerRegister) // since firmware version 1.1.1
{
  uint16_t crc16_check = 0;
  unsigned char i2cResponse[6];
  unsigned char Command[] = {0x03, 0x16, 0xA9, 0x00, 0x01, 0x91, 0x1C}; // (1 Byte) function code, (2 Bytes) Register Address, (2 Bytes) Bytes to Read, (2 Bytes) CRC CHECKSUM
  wireWrite(Command, 6, false);
  wireRead(i2cResponse, 6);
  crc16_check = (i2cResponse[5] << 8) + i2cResponse[4]; 
  if (crc16_check == calcCrc16(i2cResponse, 5))
  {
    customerRegister = (i2cResponse[2] << 8) + i2cResponse[3];
    return 0;
  }
  else
  {
    return 1;
  }
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