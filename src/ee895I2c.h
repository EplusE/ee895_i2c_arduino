/***************************************************************************/
/* headerfile for "ee895I2c.cpp" module */
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

#ifndef ee895I2c_H
#define ee895I2c_H

#include "Arduino.h"
#include "Wire.h"

// Defines
//-----------------------------------------------------------------------------
#define CRC16_ONEWIRE_START                 0xFFFF
#define FUNCTION_CODE_READ_REGISTER         0x03
#define FUNCTION_CODE_WRITE_REGISTER        0x06
#define READ_ALL_MEASUREMENTS               0x00
#define USER_REGISTER_1                     0x16A8
#define USER_REGISTER_2                     0x16A9
#define REGISTER_TEMPERATURE_CELSIUS        0x03EA
#define REGISTER_TEMPERATURE_FAHRENHEIT     0x03EC
#define REGISTER_TEMPERATURE_KELVIN         0x03F0
#define REGISTER_CO2_AVERAGE_PC             0x0424  //PC = pressure compensated
#define REGISTER_CO2_RAW_PC                 0x0426  //PC = pressure compensated
#define REGISTER_CO2_AVERAGE_NPC            0x0428  //NPC = no pressure compensated
#define REGISTER_CO2_RAW_NPC                0x042A  //NPC = no pressure compensated
#define REGISTER_PRESSURE_MBAR              0x04B0
#define REGISTER_PRESSURE_PSI               0x04B2
#define REGISTER_SERIAL_NUMBER              0x0000
#define REGISTER_FIRMWARE_VERSION           0x0008
#define REGISTER_SENSORNAME                 0x0009
#define REGISTER_MEASURING_MODE             0x01F8
#define REGISTER_MEASURING_STATUS           0x01F9
#define REGISTER_MEASURING_TRIGGER          0x01FA
#define REGISTER_DETAILED_STATUS            0x0258
#define REGISTER_CO2_MEASURING_INTERVAL     0x1450
#define REGISTER_CO2_FILTER_COEFFICIENT     0x1451
#define REGISTER_CO2_CUSTOMER_OFFSET        0x1452

// declaration of functions
//-----------------------------------------------------------------------------

class ee895I2c
{
public:
    ee895I2c(void);
    void getAllMeasurements(float &temperature, int &co2, float &pressure);
    uint8_t getTempC(float &temperature);
    uint8_t getTempF(float &temperature);
    uint8_t getTempK(float &temperature);
    uint8_t getCo2AverWithPc(int &co2); // pressure compensated 
    uint8_t getCo2RawWithPc(int &co2); // pressure compensated 
    uint8_t getCo2AverWithNpc(int &co2); // not pressure compensated
    uint8_t getCo2RawWithNpc(int &co2); // not pressure compensated
    uint8_t getPressureMbar(float &pressure); // in mbar
    uint8_t getPressurePsi(float &pressure); // in psi
    uint8_t readSerialNumber(unsigned char SerialNumber[]);
    uint8_t readFirmwareVersion(unsigned char FwVersion[]);
    uint8_t readSensorname(char Sensorname[]);
    uint8_t readMeasuringMode(bool &MeasuringMode); // 0 => continuous mode and 1 => single shot 
    uint8_t changeMeasuringMode(bool MeasuringMode); // 0 => continuous mode and 1 => single shot 
    uint8_t dataReady(bool &dataAvailable);
    uint8_t triggerReady(bool &triggerAvailable);
    uint8_t triggerNewMeasurement();  // 0 => don't care and 1 => Start new measurement cycle
    uint8_t statusDetails(uint8_t &detailedStatus); // Bit 0 = Co2 measurment too high, Bit 1 = Co2 measurment too low, Bit 2 = T measurement too high, Bit 3 = T measurement too low, Bit 6 = p measurement too high, Bit 7 p measurement too low
    uint8_t changeCo2MeasuringInterval(int measuringInterval); // min = 100 equivalent to 10 s,  max = 36000 equivalent to 1 hour 
    uint8_t readCo2MeasuringInterval(int &measuringInterval); // in 0.1 s steps
    uint8_t changeCo2FilterCoefficient(int filterCoefficient); // min = 1 equivalent to 100% step response, max = 20 equivalent to 63% step response
    uint8_t readCo2FilterCoefficient(int &filterCoefficient); // min = 1 equivalent to 100% step response, max = 20 equivalent to 63% step response
    uint8_t changeCo2CustomOffset(int customOffset); // min = -32786,  max = 32785  
    uint8_t readCo2CustomOffset(int &customOffset); // min = -32786,  max = 32785  
    uint8_t changeCustomerRegister1(int customerOffset);
    uint8_t readCustomerRegister1(int &customerRegister);
    uint8_t changeCustomerRegister2(int customerOffset);
    uint8_t readCustomerRegister2(int &customerRegister);
    unsigned char address = 0x5F;
    uint8_t writeToRegister(uint16_t registerAddress, uint8_t bytesToWrite[]);
    uint8_t readBytesFromRegister(unsigned char buf[], uint16_t registerAddress, uint16_t registerToRead, uint8_t bytesToRead);
    float bytesToValue(unsigned char buf[]);
    void wireWrite(unsigned char buf[], int to, bool stopmessage);
    void wireRead(unsigned char buf[], uint8_t to);
    uint16_t calcCrc16(unsigned char buf[], unsigned char len);
    void getErrorString(uint8_t Status, char errorString[]);
};

#endif