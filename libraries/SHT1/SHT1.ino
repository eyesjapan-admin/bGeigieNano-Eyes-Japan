/**
 * ReadSHT1xValues
 *
 * Read temperature and humidity values from an SHT1x-series (SHT10,
 * SHT11, SHT15) sensor.
 *
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au>
 * www.practicalarduino.com
 */

#include <SHT1x.h>

// Specify data and clock connections and instantiate SHT1x object
#define dataPin  A5
#define clockPin A4
SHT1x sht1x(dataPin, clockPin);

void setup()
{
   Serial.begin(9600); // Open serial connection to report values to host
   Serial.println("Starting up");
}

void loop()
{
  float temp_c;
  float temp_f;
  float humidity;

  // Read values from the sensor
  temp_c = int(sht1x.readTemperatureC());
  temp_f = int(sht1x.readTemperatureF());
  humidity = int(sht1x.readHumidity());

  // Print the values to the serial port
  Serial.print("Temperature: ");
  Serial.print(temp_c, DEC);
  Serial.print("C / ");
  Serial.print(temp_f, DEC);
  Serial.print("F. Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  delay(2000);
}
