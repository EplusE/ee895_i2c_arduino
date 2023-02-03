/*
Example script reading measurement values from the EE895 sensor via I2C interface.

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

#include <Wire.h>
#include <ee895I2c.h>
ee895I2c ee;

#define REQUEST_INTERVAL_MS 15000
#define CSV_DELIMITER ','

unsigned char SerialNumber[16];
unsigned char FwVersion[2];
char Sensorname[16];
float temperature, pressure;
int co2, Co2Offset, measuringInterval;
char errorString[200];
uint8_t errorcode;
bool MeasuringMode;

void setup()
{
  Serial.begin(9600);
  Wire.begin();                // initialize I2C peripheral (SDA..A4, SCL..A5)
  errorcode = ee.readSerialNumber(SerialNumber);
  if (errorcode != 0)
  {
    ee.getErrorString(errorcode, errorString);
    Serial.println(errorString);
  }
  else
  {
    Serial.print("Serial Number: ");
    for (int i = 0; i < 16; i++)
    {
      Serial.print(SerialNumber[i] < 16 ? "0" : "");
      Serial.print(SerialNumber[i], HEX);
    }
    Serial.println("");
  }
  errorcode = ee.readFirmwareVersion(FwVersion);
  if (errorcode != 0)
  {
    ee.getErrorString(errorcode, errorString);
    Serial.println(errorString);
  }
  else
  {
    Serial.print("Firmwareversion:");
    Serial.print(FwVersion[0]);
    Serial.print(".");
    Serial.println(FwVersion[1]);
  }
  errorcode = ee.readSensorname(Sensorname);
  if (errorcode != 0)
  {
    ee.getErrorString(errorcode, errorString);
    Serial.println(errorString);
  }
  else
  {
    Serial.print("Sensorname:");
    Serial.println(Sensorname);
  }

  ee.changeMeasuringMode(1); // 0 => continuous mode and 1 => single shot 
  errorcode = ee.readMeasuringMode(MeasuringMode);
  if (errorcode != 0)
  {
    ee.getErrorString(errorcode, errorString);
    Serial.println(errorString);
  }
  else
  {
    Serial.print("MeasuringMode:");
    Serial.println(MeasuringMode);
  }
  ee.changeCo2CustomOffset(0);
  errorcode = ee.readCo2CustomOffset(Co2Offset);
  if (errorcode != 0)
  {
    ee.getErrorString(errorcode, errorString);
    Serial.println(errorString);
  }
  else
  {
    Serial.print("Co2Offset:");
    Serial.println(Co2Offset);
  }
  ee.changeCo2MeasuringInterval(150);
  errorcode = ee.readCo2MeasuringInterval(measuringInterval);
  if (errorcode != 0)
  {
    ee.getErrorString(errorcode, errorString);
    Serial.println(errorString);
  }
  else
  {
    Serial.print("Measuring Interval:");
    Serial.println(measuringInterval);
  }
  Serial.print("temperature"); // print CSV header
  Serial.print(CSV_DELIMITER);
  Serial.print("co2");
  Serial.print(CSV_DELIMITER);
  Serial.println("pressure");
}

void loop()
{
  errorcode = ee.triggerNewMeasurement();
  if (errorcode != 0)
  {
    ee.getErrorString(errorcode, errorString);
    Serial.println(errorString);
  }
  errorcode = ee.getTempC(temperature);
  if (errorcode != 0)
  {
    ee.getErrorString(errorcode, errorString);
    Serial.println(errorString);
  }
  else
  {
    Serial.print(temperature);
    Serial.print(" °C");
  }
  errorcode = ee.getCo2AverWithPc(co2);
  if (errorcode != 0)
  {
    ee.getErrorString(errorcode, errorString);
    Serial.println(errorString);
  }
  else
  {
    Serial.print(CSV_DELIMITER);
    Serial.print(co2);
    Serial.print(" ppm");
  }
  errorcode = ee.getPressureMbar(pressure);
  if (errorcode != 0)
  {
    ee.getErrorString(errorcode, errorString);
    Serial.println(errorString);
  }
  else
  {
    Serial.print(CSV_DELIMITER);
    Serial.print(pressure);
    Serial.println(" mbar");
  }

  delay(REQUEST_INTERVAL_MS);
}
