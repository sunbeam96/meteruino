#include <ArduinoBLE.h>
#include <Adafruit_GPS.h>
#include <Arduino_LSM6DS3.h>

#define GPSSerial Serial1

Adafruit_GPS GPS(&GPSSerial);
BLEService btService("180F");
BLEUnsignedCharCharacteristic speedCharacteristic("fff1", BLEBroadcast);
BLEFloatCharacteristic accelXCharacteristic("2a3F", BLEBroadcast);
BLEFloatCharacteristic accelYCharacteristic("2b3F", BLEBroadcast);
BLEFloatCharacteristic accelZCharacteristic("183E", BLEBroadcast);
BLEFloatCharacteristic gyroXCharacteristic("183A", BLEBroadcast);
BLEFloatCharacteristic gyroYCharacteristic("2a2F", BLEBroadcast);
BLEFloatCharacteristic gyroZCharacteristic("1a3B", BLEBroadcast);

int speedKmh = 0;
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

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

  btService.addCharacteristic(speedCharacteristic);
  btService.addCharacteristic(accelXCharacteristic);
  btService.addCharacteristic(accelYCharacteristic);
  btService.addCharacteristic(accelZCharacteristic);
  btService.addCharacteristic(gyroXCharacteristic);
  btService.addCharacteristic(gyroYCharacteristic);
  btService.addCharacteristic(gyroZCharacteristic);
  BLE.addService(btService);
  BLE.setConnectable(true);

  BLE.advertise();
}

void updateReadings(){
  speedCharacteristic.writeValue(speedKmh);
  accelXCharacteristic.writeValue(accelX);
  accelYCharacteristic.writeValue(accelY);
  accelZCharacteristic.writeValue(accelZ);
  gyroXCharacteristic.writeValue(gyroX);
  gyroYCharacteristic.writeValue(gyroY);
  gyroZCharacteristic.writeValue(gyroZ);
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

  updateReadings();
}
