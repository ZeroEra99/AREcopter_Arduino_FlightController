#include "PIDcontrol.h"

PIDControl::PIDControl(double kp, double ki, double kd, double maxIntegral) : kp(kp), ki(ki), kd(kd), maxIntegral(maxIntegral)
{
    integral = 0;
    lastError = 0;
}

double PIDControl::pid(double error, double dt)
{
    integral += error * dt;
    if (integral > maxIntegral)
        integral = maxIntegral;
    else if (integral < -maxIntegral)
        integral = -maxIntegral;
    double derivative = (error - lastError) / dt;
    lastError = error;
    return kp * error + ki * integral + kd * derivative;
}