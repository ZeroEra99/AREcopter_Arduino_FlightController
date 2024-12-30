#include "Arduino.h"
#include "Receiver.h"
#include "Config.h"
#include "Debug.h"
#include "Utils.h"

PWM THROTTLE_PWM_IN(RC_INPUT_THROTTLE_PIN);
PWM ROLL_PWM_IN(RC_INPUT_ROLL_PIN);

static int THROTTLE_RAW, ROLL_RAW, PITCH_RAW, HEAD_RAW = 0;

static int cycleCounter = 0;

// Implementazione di setupRC
void setupRC(Drone *drone) {
  Serial.print("RC setup starting.\n");

  THROTTLE_PWM_IN.begin(1);
  ROLL_PWM_IN.begin(1);
  pinMode(RC_INPUT_PITCH_PIN, INPUT);
  pinMode(RC_INPUT_HEAD_PIN, INPUT);
  PITCH_RAW = 1500;
  HEAD_RAW = 1500;

  Serial.print("RC setup completed.\n");
}

// Implementazione di readRC
void readRC(Drone *drone) {
  // Lettura dei valori RAW
  cycleCounter++;
  THROTTLE_RAW = THROTTLE_PWM_IN.getValue();
  ROLL_RAW = ROLL_PWM_IN.getValue();
  if (cycleCounter == 1) PITCH_RAW = pulseIn(RC_INPUT_PITCH_PIN, HIGH, 30000);
  if (cycleCounter == (MAXCYCLE / 2) + 1) HEAD_RAW = pulseIn(RC_INPUT_HEAD_PIN, HIGH, 30000);
  if (cycleCounter == MAXCYCLE) cycleCounter = 0;

  float THROTTLE_Analog = THROTTLE_RAW;
  float ROLL_Analog = ROLL_RAW;
  float PITCH_Analog = PITCH_RAW;
  float HEAD_Analog = HEAD_RAW;

  // Controllo validità
  if (PITCH_RAW == 0 && HEAD_RAW == 0) {
    Serial.print("RC input is invalid.\n");
    drone->STATUS.isArmed = false;
    if(drone->STATUS.imuCalibrated)drone->STATUS.imuCalibrated=false;
    drone->STATUS.FAILSAFE = RC;
    return;
  }
  // Arm e Disarm
  if ((!drone->STATUS.isArmed) && THROTTLE_Analog < (PWM_MIN + ARMING_RANGE) && HEAD_Analog < (PWM_MIN + ARMING_RANGE) && PITCH_Analog < (PWM_MIN + ARMING_RANGE) && ROLL_Analog > (PWM_MAX - ARMING_RANGE)) {
    Serial.print("Drone status: armed.");
    if (drone->STATUS.FAILSAFE!=NONE) drone->STATUS.FAILSAFE = NONE;
    drone->STATUS.isArmed = true;
    HEAD_Analog = 1500;
    //delay(500); Sto delay?
  } else if (drone->STATUS.isArmed && THROTTLE_RAW < (PWM_MIN + ARMING_RANGE) && PITCH_RAW < (PWM_MAX + ARMING_RANGE) && HEAD_RAW > (PWM_MAX - ARMING_RANGE) && ROLL_Analog < (PWM_MIN + ARMING_RANGE)) {
    Serial.print("Drone status: unarmed.");
    drone->STATUS.isArmed = false;
    if(drone->STATUS.imuCalibrated)drone->STATUS.imuCalibrated=false;
    return;
  }

  // Conversione e mappatura
  ROLL_Analog = constrain(mapfloat(ROLL_Analog, PWM_MIN, PWM_MAX, -MAX_ANGLE_RC_INPUT, MAX_ANGLE_RC_INPUT), -MAX_ANGLE_RC_INPUT, MAX_ANGLE_RC_INPUT);
  PITCH_Analog = constrain(mapfloat(PITCH_Analog, PWM_MIN, PWM_MAX, -MAX_ANGLE_RC_INPUT, MAX_ANGLE_RC_INPUT), -MAX_ANGLE_RC_INPUT, MAX_ANGLE_RC_INPUT);
  HEAD_Analog = constrain(mapfloat(HEAD_Analog, PWM_MIN, PWM_MAX, -MAX_HEADING_OFFSET, MAX_HEADING_OFFSET), -MAX_HEADING_OFFSET, MAX_HEADING_OFFSET);
  THROTTLE_Analog = constrain(mapfloat(THROTTLE_Analog, PWM_MIN, PWM_MAX, IDLE_THROTTLE, MAX_THROTTLE), 0, 100);

  // Mappatura finale
  if (hypot(ROLL_Analog, PITCH_Analog) > MAX_ANGLE_RC_INPUT) {
    float scale_factor = MAX_ANGLE_RC_INPUT / hypot(ROLL_Analog, PITCH_Analog);
    ROLL_Analog *= scale_factor;
    PITCH_Analog *= scale_factor;
  }

  // Assegnamento valori
  if (drone->STATUS.isArmed) {
    drone->RC_INPUT.ROLL = ROLL_Analog;
    drone->RC_INPUT.PITCH = PITCH_Analog;
    drone->RC_INPUT.HEAD = HEAD_Analog;  //-12
    drone->RC_INPUT.THROTTLE = THROTTLE_Analog;
    /*
  if(drone->RC_INPUT.HEAD >= 360)drone->RC_INPUT.HEAD-=360;
  else if (drone->RC_INPUT.HEAD <=0)drone->RC_INPUT.HEAD+=360;
  */
  }


// Stampa i valori
#if RC_DEBUG
  DEBUG_PRINT("  Input Throttle ->");
  DEBUG_PRINT(THROTTLE_RAW);
  DEBUG_PRINT("  Input Roll ->");
  DEBUG_PRINT(ROLL_RAW);
  DEBUG_PRINT("  Input Pitch ->");
  DEBUG_PRINT(PITCH_RAW);
  DEBUG_PRINT("  Input Heading ->");
  DEBUG_PRINT(HEAD_RAW);
#endif

#if RC_GRAPH_DEBUG
  // Debug grafico per RC
  DEBUG_PRINT("RC_Throttle:");
  DEBUG_PRINT(drone->RC_INPUT.THROTTLE);
  DEBUG_PRINT(",RC_Roll:");
  DEBUG_PRINT(drone->RC_INPUT.ROLL);
  DEBUG_PRINT(",RC_Pitch:");
  DEBUG_PRINT(drone->RC_INPUT.PITCH);
  DEBUG_PRINT(",RC_Head:");
  DEBUG_PRINT(drone->RC_INPUT.HEAD);
  DEBUG_PRINTLN();
#endif

}
