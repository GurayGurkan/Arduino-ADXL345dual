/*
   Wireless (Single) Accelerometer System
   Uses 1xHC-05 and 2xADXL345 accelerometers.
   Accelerometer I2C addresses should be adjusted via SDO/ALT.ADDR pin.
   "id" parameter also selects 0x51 or 0x1D as the slave address
   Data Rate is 200 Hz.


   HC-05 Bluetooth module should be connected to pins 7(RX->TX) and 8(TX->RX).

   Author
   Guray Gurkan

   Date
   07 Jan 2018
*/

#include <ADXL345dual.h>
#include <Wire.h>



ADXL345 device;
int id = 1;



void setup() {

  device.begin(id);
  device.setDataRate(id, ADXL345_DATARATE_200HZ);
  device.setRange(id, ADXL345_RANGE_2G);



}

void loop() {


}
