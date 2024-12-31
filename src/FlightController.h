#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "PIDcontrol.h"
#include "DataStructures.h"

class FlightController
{
private:
    // PID per la stabilizzazione dell'angolo
    PIDControl pidPitchAngle = PIDControl(KP_ROLL_ANGLE, KI_ROLL_ANGLE, KD_ROLL_ANGLE, MAX_INTEGRAL);
    PIDControl pidRollAngle = PIDControl(KP_ROLL_ANGLE, KI_ROLL_ANGLE, KD_ROLL_ANGLE, MAX_INTEGRAL);
    PIDControl pidYawAngle = PIDControl(KP_YAW_ANGLE, KI_YAW_ANGLE, KD_YAW_ANGLE, MAX_INTEGRAL);
    // PID per la stabilizzazione del giroscopio
    PIDControl pidPitchGyro = PIDControl(KP_ROLL_GYRO, KI_ROLL_GYRO, KD_ROLL_GYRO, MAX_INTEGRAL);
    PIDControl pidRollGyro = PIDControl(KP_ROLL_GYRO, KI_ROLL_GYRO, KD_ROLL_GYRO, MAX_INTEGRAL);
    PIDControl pidYawGyro = PIDControl(KP_YAW_GYRO, KI_YAW_GYRO, KD_YAW_GYRO, MAX_INTEGRAL);

public:
    FlightController();
    FlightData computeData(FlightData angle, FlightData gyro, flightInput desiredAngle); // Calcola gli offset PID in base ai dati di volo
};

#endif // FLIGHTCONTROLLER_H