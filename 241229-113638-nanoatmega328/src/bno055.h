#ifndef BNO055_H
#define BNO055_H

#include "DataStructures.h"

class BNO055
{
private:
  FlightData angle; // Pitch, Roll, Yaw
  FlightData gyro;  // Angular rotation speed

public:
  BNO055();
  void bno055_setup();
  void bno055_calibrate();
  FlightData bno055_read(int flightData);
};

#endif // BNO055_H