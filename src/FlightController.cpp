#include "FlightController.h"
#include <Arduino.h>

FlightController::FlightController() : pidPitchAngle(KP_ROLL_ANGLE, KI_ROLL_ANGLE, KD_ROLL_ANGLE, MAX_INTEGRAL),
                                       pidRollAngle(KP_ROLL_ANGLE, KI_ROLL_ANGLE, KD_ROLL_ANGLE, MAX_INTEGRAL),
                                       pidYawAngle(KP_YAW_ANGLE, KI_YAW_ANGLE, KD_YAW_ANGLE, MAX_INTEGRAL),
                                       pidPitchGyro(KP_ROLL_GYRO, KI_ROLL_GYRO, KD_ROLL_GYRO, MAX_INTEGRAL),
                                       pidRollGyro(KP_ROLL_GYRO, KI_ROLL_GYRO, KD_ROLL_GYRO, MAX_INTEGRAL),
                                       pidYawGyro(KP_YAW_GYRO, KI_YAW_GYRO, KD_YAW_GYRO, MAX_INTEGRAL)
{

}

FlightData FlightController::computeData(FlightData angle, FlightData gyro, flightInput pilotData)
{
    static unsigned long tPrev = 0;   // Tempo dell'ultimo ciclo
    unsigned long t = millis();       // Ottieni il tempo corrente
    double dt = (t - tPrev) / 1000.0; // Calcola il tempo trascorso in secondi
    tPrev = t;

    FlightData angleError;
    angleError.pitch = angle.pitch - pilotData.pitch;
    angleError.roll = angle.roll - pilotData.roll;
    angleError.yaw = angle.yaw - pilotData.yaw;
    FlightData gyroError;
    // PID di primo livello per l'angolo-
    gyroError.pitch = pidPitchAngle.pid(angleError.pitch, dt) - gyro.pitch;
    gyroError.roll = pidRollAngle.pid(angleError.roll, dt) - gyro.roll;
    gyroError.yaw = pidYawAngle.pid(angleError.yaw, dt) - gyro.yaw;
    // PID di secondo livello per il giroscopio (Calcolato con l'ooutput del PID di primo livello)
    FlightData pidOffset;
    pidOffset.pitch = pidPitchGyro.pid(gyroError.pitch, dt);
    pidOffset.roll = pidRollGyro.pid(gyroError.roll, dt);
    pidOffset.yaw = pidYawGyro.pid(gyroError.yaw, dt);

    return pidOffset;
}
