/*
Example script reading measurement values from the EE895 sensor via I2C interface.

Copyright 2021 E+E Elektronik Ges.m.b.H.

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

#define REQUEST_INTERVAL_MS 15000
#define CSV_DELIMITER ','

unsigned char i2cResponse[8];
float co2, temperature, pressure;
int i = 0;

void setup()
{
  Serial.begin(9600);
  Wire.begin();                     // initialize I2C peripheral (SDA..A4, SCL..A5)
  Serial.print("temperature");                // print CSV header
  Serial.print(CSV_DELIMITER);
  Serial.print("co2");
  Serial.print(CSV_DELIMITER);
  Serial.println("pressure");
}


void loop()
{
  i = 0;
  Wire.beginTransmission(0x5E);     // set register address 0 an simple I2C interface
  Wire.write(0);
  Wire.endTransmission(true);
  // read data from slave device
  Wire.requestFrom(0x5E, 8, true);
  while (Wire.available())
  {
    i2cResponse[i++] = Wire.read();
  }
  // check if all requested data could be read
  if (i >= 8)
  {
    temperature = (float)(i2cResponse[2] * 256 + i2cResponse[3]) / 100;
    co2 = (float)(i2cResponse[0] * 256 + i2cResponse[1]);
    pressure = (float)(i2cResponse[6] * 256 + i2cResponse[7]) / 10;
    Serial.print(temperature); 
    Serial.print(" °C");
    Serial.print(CSV_DELIMITER);
    Serial.print(co2);
    Serial.print(" ppm");
    Serial.print(CSV_DELIMITER);
    Serial.print(pressure);
    Serial.println(" mbar");
  }
  else
  {
    Serial.println("Error - not able to read measurement data");
  }
  
  delay(REQUEST_INTERVAL_MS);
}
