#include <ArduinoBLE.h>
#include <Adafruit_GPS.h>
#include <Arduino_LSM6DS3.h>
#include <avr/dtostrf.h>

#define GPSSerial Serial1

Adafruit_GPS GPS(&GPSSerial);
BLEService teleBtService("00002000-0000-1000-8000-00805f9b34fb");
char bleBuffer[27] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//char testBuffer[3] = {1, 1, 1};
BLECharacteristic collectiveGpsAndImuCharacteristic_("00002137-0000-1000-8000-00805f9b34fb", BLERead, bleBuffer);

float speedKmh = 0.0f;
float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;

void setup() {
  while (!Serial);
  Serial.begin(115200);
  GPS.begin(9600);

  if (!BLE.begin()) {
    Serial.println("BLE init failure!");
    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("IMU init failure!");
    while (1);
  }

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  BLE.setDeviceName("Telemetruino-A563221");
  BLE.setLocalName("Telemetruino-A563221");
  BLE.setConnectionInterval(0x0001, 0x0c80); // 1.25 ms minimum, 4 s maximum
  BLE.setAdvertisedService(teleBtService);

  teleBtService.addCharacteristic(collectiveGpsAndImuCharacteristic_);
  BLE.addService(teleBtService);
  BLE.setConnectable(true);
}

void composePayload(){
  //GPS
  char speedBuffer[3];
  dtostrf(speedKmh, -3, 0, speedBuffer);
  for (int i=0; i<3; i++)
  {
    bleBuffer[i] = speedBuffer[i];
  }
  //ACCELEROMETER
  char accelXBuffer[4];
  char accelYBuffer[4];
  char accelZBuffer[4];
  dtostrf(accelX, -4, 1, accelXBuffer);
  dtostrf(accelY, -4, 1, accelYBuffer);
  dtostrf(accelZ, -4, 1, accelZBuffer);
  for (int i=0; i<4; i++)
  {
    bleBuffer[3 + i] = accelXBuffer[i];
    bleBuffer[7 + i] = accelXBuffer[i];
    bleBuffer[11 + i] = accelXBuffer[i];
  }
  //GYROSCOPE
  char gyroXBuffer[4];
  char gyroYBuffer[4];
  char gyroZBuffer[4];
  dtostrf(gyroX, -4, 0, gyroXBuffer);
  dtostrf(gyroY, -4, 0, gyroYBuffer);
  dtostrf(gyroZ, -4, 0, gyroZBuffer);
  for (int i=0; i<4; i++)
  {
    bleBuffer[15 + i] = gyroXBuffer[i];
    bleBuffer[19 + i] = gyroYBuffer[i];
    bleBuffer[23 + i] = gyroZBuffer[i];
  }
  collectiveGpsAndImuCharacteristic_.writeValue(bleBuffer, 27, false);
}

void updateReadings(){
  BLE.advertise();
}

void loop() {
  if (GPS.fix)
    speedKmh = GPS.speed*1.852;
  else
    Serial.println("Waiting for GPS fix...");

  if (IMU.accelerationAvailable())
    IMU.readAcceleration(accelX, accelY, accelZ);
  if (IMU.gyroscopeAvailable())
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  composePayload();
  updateReadings();
}
