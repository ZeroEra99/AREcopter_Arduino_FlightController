#ifndef CONTROL_H
#define CONTROL_H

#include "drone.h"

// Definizioni delle costanti
#define MAX_DIFF_INPUT_PITCH 8
#define MAX_DIFF_INPUT_ROLL 8
#define PID_INTEGRAL_ERROR 2
#define MAX_OFFSET 40

// Struttura del PID
struct pid {
  float P;
  float I;
  float D;
};

// Dichiarazioni dei PID per Roll, Pitch e Yaw
extern const pid PID_VALUE_ROLL;
extern const pid PID_VALUE_PITCH;
extern const pid PID_VALUE_YAW;

// Variabili per il controllo e differenze degli input
extern ControlInput DIFF_INPUT;
extern ControlInput LAST_DIFF_INPUT;

// Funzione per il controllo PID
void PID(Drone *drone, float elapsedTime);

#endif // CONTROL_H
