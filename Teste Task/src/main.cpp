#include <iostream>
#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C

// MPU6050 Constants
const int MPU = 0x68;
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// GPS Constants
int RXPin = 16;
int TXPin = 17;
int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

void dmpu (void * pvParameters)
{
  while(1)
  {
    // MPU6050 Sensor Reading
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(0);
  Wire.requestFrom(MPU, 14, 1);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  Serial.print(" AcX = ");
  Serial.print(AcX);
  Serial.print(" | AcY = ");
  Serial.print(AcY);
  Serial.print(" | AcZ = ");
  Serial.print(AcZ);
  Serial.print(" | Tmp = ");
  Serial.print(Tmp / 340.00 + 36.53);
  Serial.print(" | GyX = ");
  Serial.print(GyX);
  Serial.print(" | GyY = ");
  Serial.print(GyY);
  Serial.print(" | GyZ = ");
  Serial.println(GyZ);
  delay(300);
  }
}

void dbmp (void * pvParameters)
{
  while(1)
  {
    // BMP280 Sensor Reading
  if (bmp.takeForcedMeasurement()) {
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25));
    Serial.println(" m");

    Serial.println();
    delay(2000);
  } else {
    Serial.println("Forced measurement failed!");
  }

  }
}



void displayGPSInfo() {
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Não detectamos a localização");
  }

  Serial.print("Data: ");
  if (gps.date.isValid()) {
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.println(gps.date.year());
  } else {
    Serial.println("Erro");
  }

  Serial.print("Time: ");
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour() - 3);
    Serial.print(":");
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
  } else {
    Serial.println("Não detectamos o horário atual");
  }

  Serial.println();
  Serial.println();
  delay(1000);
}

void dgps (void * pvParameters)
{
  while(1)
  {
      // GPS Reading
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      displayGPSInfo();
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("Sinal GPS não detectado");
    while (true) delay(10);
  }
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(1);

  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);

  gpsSerial.begin(GPSBaud);

  xTaskCreate(dmpu, "dados mpu", 3000, NULL, 1, NULL);
  xTaskCreate(dbmp, "dados bmp", 3000, NULL, 1, NULL);
  xTaskCreate (dgps, "dados gps", 3000, NULL, 1, NULL);
}

void loop()
{

}