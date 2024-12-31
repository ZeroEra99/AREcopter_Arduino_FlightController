#include "bno055.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno055 = Adafruit_BNO055(55, 0x29); // Oggetto per il sensore BNO055
FlightData flightDataError = {INVALID, INVALID, INVALID};

BNO055::BNO055()
{
}

void BNO055::setup()
{
  // Configurazione della IMU
  Serial.print("IMU setup starting.\n");
  if (!bno055.begin())
  {
    Serial.print("IMU setup failed.\n");
    while (1)
      ; // Entra in loop infinito se l'inizializzazione fallisce
  }
  Serial.print("IMU setup complete.\n");
}

void BNO055::calibrate()
{
  // Calibrate IMU sensor
}

FlightData BNO055::read(int flightData)
{
  // Leggi i dati di volo (angolo o giroscopio)
  switch (flightData)
  {
  case ANGLE:
    sensors_event_t angle_event;
    if (!bno055.getEvent(&angle_event, Adafruit_BNO055::VECTOR_EULER))
    {
      Serial.print("Error reading angle data.\n");
      // Failsafe ?
      return flightDataError;
    }
    angle.pitch = angle_event.orientation.pitch;
    angle.roll = angle_event.orientation.roll;
    angle.yaw = angle_event.orientation.heading;
    return angle;
  case GYRO:
    sensors_event_t gyro_event;
    if (!bno055.getEvent(&gyro_event, Adafruit_BNO055::VECTOR_GYROSCOPE))
    {
      Serial.print("Error reading gyro data.\n");
      // Failsafe ?
      return flightDataError;
    }
    gyro.pitch = gyro_event.gyro.pitch;
    gyro.roll = gyro_event.gyro.roll;
    gyro.yaw = gyro_event.gyro.heading;
    return gyro;
  }
  Serial.print("Invalid flight data type.\n");
  // Failsafe ?
  return flightDataError;
}