
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

void setup() {
  delay(1000);
  Serial.begin(115200);
  delay(500);
  Serial.println("Start!");
  
  Wire.begin(18, 46);
  
  bool status;
  status = bme.begin(0x77);  
  if (!status) {
    Serial.println("Could not find a valid sensor, Please check wiring!");
    while (1);
  }
  Serial.println("-- Test --");
}
void loop() { 
  printValues();
  delay(2000);
}

void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");
  Serial.print("Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  Serial.println();
}