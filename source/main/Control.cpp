#include "Arduino.h"
#include "Control.h"
#include "Config.h"
#include "Utils.h"
#include "Debug.h"

// Variabili per le differenze di input e l'ultimo input
// Angolo
ControlInput DIFF_INPUT_ANGLE;
ControlInput LAST_DIFF_INPUT_ANGLE;
// Velocità rotazione
ControlInput DIFF_INPUT_RATE;
ControlInput LAST_DIFF_INPUT_RATE;

// Variabili statiche per l'accumulo della componente integrale
// Angolo
static float integralRollAngle = 0;
static float integralPitchAngle = 0;
static float integralHeadAngle = 0;
// Velocità rotazione
static float integralRollRate = 0;
static float integralPitchRate = 0;
static float integralHeadRate = 0;

// Variabili di Desired Rate
float DesiredRateRoll = 0.0;   // Setpoint della velocità angolare per Roll
float DesiredRatePitch = 0.0;  // Setpoint della velocità angolare per Pitch
float DesiredRateHead = 0.0;

unsigned long lastTime = 0;  // Variabile per il tempo dell'ultimo ciclo

// Funzione per calcolare il PID
float calculatePID(float throttle, float error, float integral, float prevError, float kp, float ki, float kd, float elapsedTime, float maxProportional, float maxIntegral, float maxDerivative) {
  float P, I, D;
  P = error * kp;
  constrain(P, -maxProportional, maxProportional);
  if (throttle > IDLE_THROTTLE) {
    integral += error * elapsedTime;
    integral = constrain(integral, -maxIntegral, maxIntegral);
    I = integral * ki;
  } else I = 0;
  D = kd * ((error - prevError) / elapsedTime);
  constrain(D, -maxDerivative, maxDerivative);
  return P + I + D;
}

// Funzione per il controllo PID
void PID(Drone *drone) {
  // Offset PID Angolo
  pid PID_OFFSET_ROLL_ANGLE, PID_OFFSET_PITCH_ANGLE, PID_OFFSET_HEAD_ANGLE;
  // Offset PID Velocità rotazione
  pid PID_OFFSET_ROLL_RATE, PID_OFFSET_PITCH_RATE, PID_OFFSET_HEAD_RATE;
  // Output PID
  ControlInput ESC_OFFSET;
  // Output ESC
  ESC_output ESC_OFFSET_SUM, ESC_OUTPUT;

  unsigned long currentTime = millis();                   // Ottieni il tempo corrente in millisecondi
  float elapsedTime = (currentTime - lastTime) / 1000.0;  // Calcola il tempo trascorso in secondi
  lastTime = currentTime;                                 // Aggiorna lastTime per il prossimo ciclo

  /* Livello lineare - Calcola i DesiredRate */
  // Calcola le differenze tra i valori di input della IMU e del RC (Angolo)
  DIFF_INPUT_ANGLE.ROLL = drone->IMU_INPUT.ROLL - drone->RC_INPUT.ROLL;
  DIFF_INPUT_ANGLE.PITCH = drone->IMU_INPUT.PITCH - drone->RC_INPUT.PITCH;
  DIFF_INPUT_ANGLE.HEAD = drone->RC_INPUT.HEAD;  //constrain(headingDiff, -MAX_DIFF_INPUT_HEAD, MAX_DIFF_INPUT_HEAD);
  // Calcolo dei PID
  DesiredRateRoll = calculatePID(drone->RC_INPUT.THROTTLE, DIFF_INPUT_ANGLE.ROLL, integralRollAngle, LAST_DIFF_INPUT_ANGLE.ROLL, PID_VALUE_P_ANGLE, PID_VALUE_I_ANGLE, PID_VALUE_D_ANGLE, elapsedTime, MAX_PROPORTIONAL_ANGLE, MAX_INTEGRAL_ANGLE, MAX_DERIVATIVE_ANGLE);
  DesiredRatePitch = calculatePID(drone->RC_INPUT.THROTTLE, DIFF_INPUT_ANGLE.PITCH, integralPitchAngle, LAST_DIFF_INPUT_ANGLE.PITCH, PID_VALUE_P_ANGLE, PID_VALUE_I_ANGLE, PID_VALUE_D_ANGLE, elapsedTime, MAX_PROPORTIONAL_ANGLE, MAX_INTEGRAL_ANGLE, MAX_DERIVATIVE_ANGLE);
  DesiredRateHead = calculatePID(drone->RC_INPUT.THROTTLE, DIFF_INPUT_ANGLE.HEAD, integralHeadAngle, LAST_DIFF_INPUT_ANGLE.HEAD, PID_VALUE_HEAD_P_ANGLE, PID_VALUE_HEAD_I_ANGLE, PID_VALUE_HEAD_D_ANGLE, elapsedTime, MAX_PROPORTIONAL_HEAD_ANGLE, MAX_INTEGRAL_HEAD_ANGLE, MAX_DERIVATIVE_HEAD_ANGLE);
  // Salvataggio dati attuali
  LAST_DIFF_INPUT_ANGLE.ROLL = DIFF_INPUT_ANGLE.ROLL;
  LAST_DIFF_INPUT_ANGLE.PITCH = DIFF_INPUT_ANGLE.PITCH;
  LAST_DIFF_INPUT_ANGLE.HEAD = DIFF_INPUT_ANGLE.HEAD;

  /* Livello velocità di rotazione - Calcola gli offset */
  // Calcola le differenze tra DesiredRate e velocità angolari misurate
  DIFF_INPUT_RATE.ROLL = DesiredRateRoll - drone->IMU_INPUT.ROLL_RATE;
  DIFF_INPUT_RATE.PITCH = DesiredRatePitch - drone->IMU_INPUT.PITCH_RATE;
  DIFF_INPUT_RATE.HEAD = DesiredRateHead - drone->IMU_INPUT.HEAD_RATE;
  // Calcolo dei PID
  ESC_OFFSET.ROLL = calculatePID(drone->RC_INPUT.THROTTLE, DIFF_INPUT_RATE.ROLL, integralRollRate, LAST_DIFF_INPUT_RATE.ROLL, PID_VALUE_P_RATE, PID_VALUE_I_RATE, PID_VALUE_D_RATE, elapsedTime, MAX_PROPORTIONAL_RATE, MAX_INTEGRAL_RATE, MAX_DERIVATIVE_RATE);
  ESC_OFFSET.PITCH = calculatePID(drone->RC_INPUT.THROTTLE, DIFF_INPUT_RATE.PITCH, integralPitchRate, LAST_DIFF_INPUT_RATE.PITCH, PID_VALUE_P_RATE, PID_VALUE_I_RATE, PID_VALUE_D_RATE, elapsedTime, MAX_PROPORTIONAL_RATE, MAX_INTEGRAL_RATE, MAX_DERIVATIVE_RATE);
  ESC_OFFSET.HEAD = calculatePID(drone->RC_INPUT.THROTTLE, DIFF_INPUT_RATE.HEAD, integralHeadRate, LAST_DIFF_INPUT_RATE.HEAD, PID_VALUE_HEAD_P_RATE, PID_VALUE_HEAD_I_RATE, PID_VALUE_HEAD_D_RATE, elapsedTime, MAX_PROPORTIONAL_HEAD_RATE, MAX_INTEGRAL_HEAD_RATE, MAX_DERIVATIVE_HEAD_RATE);
  // Aggiorna i valori precedenti per le velocità angolari
  LAST_DIFF_INPUT_RATE.ROLL = DIFF_INPUT_RATE.ROLL;
  LAST_DIFF_INPUT_RATE.PITCH = DIFF_INPUT_RATE.PITCH;

  /* Calcolo degli offset */
  // Proporzionalità rispetto a Throttle
  ESC_OFFSET.ROLL *= drone->RC_INPUT.THROTTLE / PID_PROPORZIONALITA_THROTTLE;
  ESC_OFFSET.PITCH *= drone->RC_INPUT.THROTTLE / PID_PROPORZIONALITA_THROTTLE;
  ESC_OFFSET.HEAD *= drone->RC_INPUT.THROTTLE / PID_PROPORZIONALITA_THROTTLE;
  // Somma degli offset
  ESC_OFFSET_SUM.FRL = drone->RC_INPUT.THROTTLE - ESC_OFFSET.ROLL + ESC_OFFSET.PITCH - ESC_OFFSET.HEAD;
  ESC_OFFSET_SUM.FRR = drone->RC_INPUT.THROTTLE + ESC_OFFSET.ROLL + ESC_OFFSET.PITCH + ESC_OFFSET.HEAD;
  ESC_OFFSET_SUM.BKL = drone->RC_INPUT.THROTTLE - ESC_OFFSET.ROLL - ESC_OFFSET.PITCH + ESC_OFFSET.HEAD;
  ESC_OFFSET_SUM.BKR = drone->RC_INPUT.THROTTLE + ESC_OFFSET.ROLL - ESC_OFFSET.PITCH - ESC_OFFSET.HEAD;

  /* Mappatura finale */
  drone->ESC_OUTPUT.FRL = mapfloat(ESC_OFFSET_SUM.FRL, 0, 100, PWM_MIN, PWM_MAX);
  drone->ESC_OUTPUT.FRR = mapfloat(ESC_OFFSET_SUM.FRR, 0, 100, PWM_MIN, PWM_MAX);
  drone->ESC_OUTPUT.BKL = mapfloat(ESC_OFFSET_SUM.BKL, 0, 100, PWM_MIN, PWM_MAX);
  drone->ESC_OUTPUT.BKR = mapfloat(ESC_OFFSET_SUM.BKR, 0, 100, PWM_MIN, PWM_MAX);

  // Stampa i valori
#if PID_DEBUG
  DEBUG_PRINT(", PID ANGLE Roll -> P: ");
  DEBUG_PRINT(PID_OFFSET_ROLL_ANGLE.P);
  DEBUG_PRINT(", I: ");
  DEBUG_PRINT(PID_OFFSET_ROLL_ANGLE.I);
  DEBUG_PRINT(", D: ");
  DEBUG_PRINT(PID_OFFSET_ROLL_ANGLE.D);
  DEBUG_PRINT(", DesiredRateRoll: ");
  DEBUG_PRINT(DesiredRateRoll);
  /*
  DEBUG_PRINT(", PID ANGLE Pitch -> P: ");
  DEBUG_PRINT(PID_OFFSET_PITCH_ANGLE.P);
  DEBUG_PRINT(", I: ");
  DEBUG_PRINT(PID_OFFSET_PITCH_ANGLE.I);
  DEBUG_PRINT(", D: ");
  DEBUG_PRINT(PID_OFFSET_PITCH_ANGLE.D);
  DEBUG_PRINT(", DesiredRatePitch: ");
  DEBUG_PRINT(DesiredRatePitch);

  DEBUG_PRINT(", PID ANGLE Head -> P: ");
  DEBUG_PRINT(PID_OFFSET_HEAD_ANGLE.P);
  DEBUG_PRINT(", I: ");
  DEBUG_PRINT(PID_OFFSET_HEAD_ANGLE.I);
  DEBUG_PRINT(", D: ");
  DEBUG_PRINT(PID_OFFSET_HEAD_ANGLE.D);
  DEBUG_PRINT(", DesiredRateHead: ");
  DEBUG_PRINT(DesiredRateHead);*/

  DEBUG_PRINT(", PID RATE Roll -> P: ");
  DEBUG_PRINT(PID_OFFSET_ROLL_RATE.P);
  DEBUG_PRINT(", I: ");
  DEBUG_PRINT(PID_OFFSET_ROLL_RATE.I);
  DEBUG_PRINT(", D: ");
  DEBUG_PRINT(PID_OFFSET_ROLL_RATE.D);
  /*
  DEBUG_PRINT(", PID RATE Pitch -> P: ");
  DEBUG_PRINT(PID_OFFSET_PITCH_RATE.P);
  DEBUG_PRINT(", I: ");
  DEBUG_PRINT(PID_OFFSET_PITCH_RATE.I);
  DEBUG_PRINT(", D: ");
  DEBUG_PRINT(PID_OFFSET_PITCH_RATE.D);

  DEBUG_PRINT(", PID RATE Head -> P: ");
  DEBUG_PRINT(PID_OFFSET_HEAD_RATE.P);
  DEBUG_PRINT(", I: ");
  DEBUG_PRINT(PID_OFFSET_HEAD_RATE.I);
  DEBUG_PRINT(", D: ");
  DEBUG_PRINT(PID_OFFSET_HEAD_RATE.D);
*/
  DEBUG_PRINT(", ESC Offsets -> Roll: ");
  /*
  DEBUG_PRINT(ESC_OFFSET.ROLL);
  DEBUG_PRINT(", Pitch: ");
  DEBUG_PRINT(ESC_OFFSET.PITCH);
  DEBUG_PRINT(", Head: ");
  DEBUG_PRINT(ESC_OFFSET.HEAD);*/
#endif

#if PID_GRAPH_DEBUG
  // Debug grafico dei valori PID
  DEBUG_PRINT("PID_ANGLE_Roll,P:");
  DEBUG_PRINT(PID_OFFSET_ROLL_ANGLE.P);
  DEBUG_PRINT(",I:");
  DEBUG_PRINT(PID_OFFSET_ROLL_ANGLE.I);
  DEBUG_PRINT(",D:");
  DEBUG_PRINT(PID_OFFSET_ROLL_ANGLE.D);
  DEBUG_PRINT(",DesiredRateRoll:");
  DEBUG_PRINT(DesiredRateRoll);

  DEBUG_PRINT(",PID_ANGLE_Pitch,P:");
  DEBUG_PRINT(PID_OFFSET_PITCH_ANGLE.P);
  DEBUG_PRINT(",I:");
  DEBUG_PRINT(PID_OFFSET_PITCH_ANGLE.I);
  DEBUG_PRINT(",D:");
  DEBUG_PRINT(PID_OFFSET_PITCH_ANGLE.D);
  DEBUG_PRINT(",DesiredRatePitch:");
  DEBUG_PRINT(DesiredRatePitch);

  DEBUG_PRINT(",PID_ANGLE_Head,P:");
  DEBUG_PRINT(PID_OFFSET_HEAD_ANGLE.P);
  DEBUG_PRINT(",I:");
  DEBUG_PRINT(PID_OFFSET_HEAD_ANGLE.I);
  DEBUG_PRINT(",D:");
  DEBUG_PRINT(PID_OFFSET_HEAD_ANGLE.D);
  DEBUG_PRINT(",DesiredRateHead:");
  DEBUG_PRINT(DesiredRateHead);

  DEBUG_PRINT(",PID_RATE_Roll,P:");
  DEBUG_PRINT(PID_OFFSET_ROLL_RATE.P);
  DEBUG_PRINT(",I:");
  DEBUG_PRINT(PID_OFFSET_ROLL_RATE.I);
  DEBUG_PRINT(",D:");
  DEBUG_PRINT(PID_OFFSET_ROLL_RATE.D);

  DEBUG_PRINT(",PID_RATE_Pitch,P:");
  DEBUG_PRINT(PID_OFFSET_PITCH_RATE.P);
  DEBUG_PRINT(",I:");
  DEBUG_PRINT(PID_OFFSET_PITCH_RATE.I);
  DEBUG_PRINT(",D:");
  DEBUG_PRINT(PID_OFFSET_PITCH_RATE.D);

  DEBUG_PRINT(",PID_RATE_Head,P:");
  DEBUG_PRINT(PID_OFFSET_HEAD_RATE.P);
  DEBUG_PRINT(",I:");
  DEBUG_PRINT(PID_OFFSET_HEAD_RATE.I);
  DEBUG_PRINT(",D:");
  DEBUG_PRINT(PID_OFFSET_HEAD_RATE.D);

  DEBUG_PRINT(",ESC_Offsets,Roll:");
  DEBUG_PRINT(ESC_OFFSET.ROLL);
  DEBUG_PRINT(",Pitch:");
  DEBUG_PRINT(ESC_OFFSET.PITCH);
  DEBUG_PRINT(",Head:");
  DEBUG_PRINT(ESC_OFFSET.HEAD);
  DEBUG_PRINTLN();
#endif
}
