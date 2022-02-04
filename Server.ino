
#include <SPI.h>
#include <icp101xx.h> // ICP sensor library

ICP101xx mysensor;

void setup() {
  //INITIALIZE THE ICP-101xx SENSOR
  mysensor.begin(); //sensor will use the Ardino "Wire" object for I2C by default
  if (!mysensor.begin()) {
    Serial.println("Sensor setup failed.");
    while (1);
  }
}

float packetnum = 0; // packet counter, increments each data transmission

void loop() {
  if (mysensor.isConnected()) {
    mysensor.measure(mysensor.FAST); //FAST - Low-power mode, 3ms, ±3.2 Pa noise
  } else { // ICP sensor is not connected
    Serial.println("Pressure sensor is disconnected!");
    while (1);
  }
}
