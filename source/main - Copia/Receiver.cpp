#include "Arduino.h"
#include "Receiver.h"
#include "Config.h"
#include "Debug.h"
#include "Utils.h"

#if TEST_LETTURA_LIBRERIA
PWM THR_PWM_IN(RC_INPUT_THROTTLE_PIN);
PWM ROLL_PWM_IN(RC_INPUT_ROLL_PIN);
#endif
static int THR_RAW,ROLL_RAW,PITCH_RAW,HEAD_RAW = 0;

static int cycleCounter = 0;

// Implementazione di setupRC
void setupRC(Drone *drone) {
  Serial.print("RC setup starting.\n");

  #if TEST_LETTURA_LIBRERIA // Configurazione dei pin tramite libreria PWM
    THR_PWM_IN.begin(1);
    ROLL_PWM_IN.begin(1);
  #elif !TEST_LETTURA_LIBRERIA // Configurazione standard
    pinMode(RC_INPUT_THROTTLE_PIN, INPUT);
    pinMode(RC_INPUT_ROLL_PIN, INPUT);
  #endif
  pinMode(RC_INPUT_PITCH_PIN, INPUT);
  pinMode(RC_INPUT_HEAD_PIN, INPUT);

  PITCH_RAW = 1500;
  HEAD_RAW = 1500;
  
  drone->RC_INPUT.HEAD = 0;

  Serial.print("RC setup completed.\n");
}

// Implementazione di readRC
void readRC(Drone *drone) {
  // Lettura dei valori RAW
  int preRead = millis();

  cycleCounter++;
  #if TEST_LETTURA_LIBRERIA
  THR_RAW = THR_PWM_IN.getValue();
  ROLL_RAW = ROLL_PWM_IN.getValue();
  #elif !TEST_LETTURA_LIBRERIA
    THR_RAW = pulseIn(RC_INPUT_THROTTLE_PIN, HIGH, 30000);
    ROLL_RAW = pulseIn(RC_INPUT_ROLL_PIN, HIGH, 30000);
  #endif
  
  #if !TEST_NO_PULSEIN
    #if TEST_CICLI_LETTURA
      if(cycleCounter==1)PITCH_RAW = pulseIn(RC_INPUT_PITCH_PIN, HIGH, 30000);
      if(cycleCounter==(MAXCYCLE/2)+1)HEAD_RAW = pulseIn(RC_INPUT_HEAD_PIN, HIGH, 30000);
    #elif !TEST_CICLI_LETTURA
      PITCH_RAW = pulseIn(RC_INPUT_PITCH_PIN, HIGH, 30000);
      HEAD_RAW = pulseIn(RC_INPUT_HEAD_PIN, HIGH, 30000);
    #endif
    if(cycleCounter==MAXCYCLE)cycleCounter=0;
  #elif TEST_NO_PULSEIN
    PITCH_RAW = 1500;
    HEAD_RAW = 1500;
  #endif

  float THR_PulseIn = THR_RAW;
  float ROL_PulseIn = ROLL_RAW;
  float PIT_PulseIn = PITCH_RAW;
  float HEAD_PulseIn = HEAD_RAW;

  int postRead = millis() - preRead;

  // Controllo validità
  int preVal = millis();
  if (ROL_PulseIn == 0 || PIT_PulseIn == 0 || HEAD_PulseIn == 0) {
    Serial.print("RC input is invalid.\n");
    drone->STATUS.FAILSAFE = RC;
  }
  int postVal = millis() - preVal;

  // Arm e Disarm
  int preArm = millis();
  if ((!drone->STATUS.isArmed) &&
      THR_RAW < (IO_MIN + ARMING_RANGE) &&
      HEAD_RAW < (IO_MIN + ARMING_RANGE) &&
      PITCH_RAW < (IO_MIN + ARMING_RANGE) &&
      ROLL_RAW > (IO_MAX - ARMING_RANGE)) {
    Serial.print("Drone status: armed.");
    drone->STATUS.isArmed = true;
    drone->RC_INPUT.HEAD = 0;
    HEAD_PulseIn = 1500;
    delay(500);
  } else if (drone->STATUS.isArmed &&
            THR_RAW < (IO_MIN + ARMING_RANGE) &&
             PITCH_RAW < (IO_MAX + ARMING_RANGE) &&
             HEAD_RAW > (IO_MAX - ARMING_RANGE) &&
             ROLL_RAW < (IO_MIN + ARMING_RANGE)) {
    Serial.print("Drone status: unarmed.");
    drone->STATUS.isArmed = false;
  }
  int postArm = millis() - preArm;

  // Conversione e mappatura
  int preConv = millis();
  ROL_PulseIn = constrain(mapfloat(ROL_PulseIn, IO_MIN, IO_MAX, -MAX_RC_INPUT_ROLL, MAX_RC_INPUT_ROLL), -MAX_RC_INPUT_ROLL, MAX_RC_INPUT_ROLL);
  PIT_PulseIn = constrain(mapfloat(PIT_PulseIn, IO_MIN, IO_MAX, -MAX_RC_INPUT_PITCH, MAX_RC_INPUT_PITCH), -MAX_RC_INPUT_PITCH, MAX_RC_INPUT_PITCH);
  HEAD_PulseIn = constrain(mapfloat(HEAD_PulseIn, IO_MIN, IO_MAX, -MAX_RC_INPUT_HEAD, MAX_RC_INPUT_HEAD), -MAX_RC_INPUT_HEAD, MAX_RC_INPUT_HEAD);
  THR_PulseIn = constrain(mapfloat(THR_PulseIn, IO_MIN, IO_MAX, 0, MAX_RC_INPUT_THROTTLE), 0, MAX_RC_INPUT_THROTTLE);
  int postConv = millis() - preConv;

  // Smoothing
  int preSmooth = millis();
  if (abs(ROL_PulseIn) < MIN_RC_INPUT_ROLL) ROL_PulseIn = 0;
  if (abs(PIT_PulseIn) < MIN_RC_INPUT_PITCH) PIT_PulseIn = 0;
  if (abs(HEAD_PulseIn) < MIN_RC_INPUT_HEAD) HEAD_PulseIn = 0;
  if (abs(THR_PulseIn) < MIN_RC_INPUT_THROTTLE) THR_PulseIn = 0;
  int postSmooth = millis() - preSmooth;

  // Mappatura finale
  int preMap = millis();
  if (hypot(ROL_PulseIn, PIT_PulseIn) > MAX_RC_INPUT_2AXIS_COMBINED) {
      float scale_factor = MAX_RC_INPUT_2AXIS_COMBINED / hypot(ROL_PulseIn, PIT_PulseIn);
      ROL_PulseIn *= scale_factor;
      PIT_PulseIn *= scale_factor;
  }
  int postMap = millis() - preMap;

  // Assegnamento valori
  int preAssign = millis();
  if(drone->STATUS.isArmed){
  drone->RC_INPUT.ROL = ROL_PulseIn;
  drone->RC_INPUT.PIT = PIT_PulseIn;
  drone->RC_INPUT.HEAD = HEAD_PulseIn-12;//+= HEAD_PulseIn;
  drone->RC_INPUT.THR = THR_PulseIn;

  /*
  if(drone->RC_INPUT.HEAD >= 360)drone->RC_INPUT.HEAD-=360;
  else if (drone->RC_INPUT.HEAD <=0)drone->RC_INPUT.HEAD+=360;
  */
  }
  
  int postAssign = millis() - preAssign;

  // Stampa i valori
  #if RC_DEBUG
    DEBUG_PRINT("  Input Throttle ->");
    DEBUG_PRINT(drone->RC_INPUT.THR);
    DEBUG_PRINT("  Input Roll ->");
    DEBUG_PRINT(drone->RC_INPUT.ROL);
    DEBUG_PRINT("  Input Pitch ->");
    DEBUG_PRINT(drone->RC_INPUT.PIT);
    DEBUG_PRINT("  Input Heading ->");
    DEBUG_PRINT(drone->RC_INPUT.HEAD);
  #endif

  #if RC_GRAPH_DEBUG
    // Debug grafico per RC
    DEBUG_PRINT("RC_Throttle:"); DEBUG_PRINT(drone->RC_INPUT.THR);
    DEBUG_PRINT(",RC_Roll:"); DEBUG_PRINT(drone->RC_INPUT.ROL);
    DEBUG_PRINT(",RC_Pitch:"); DEBUG_PRINT(drone->RC_INPUT.PIT);
    DEBUG_PRINT(",RC_Head:"); DEBUG_PRINT(drone->RC_INPUT.HEAD);
    DEBUG_PRINTLN();
  #endif

  #if RC_EXTIME_DEBUG
    DEBUG_PRINT("  Read:");DEBUG_PRINT(postRead);
    DEBUG_PRINT("  Validation:");DEBUG_PRINT(postVal);
    DEBUG_PRINT("  Arming check:");DEBUG_PRINT(postArm);
    DEBUG_PRINT("  Conversion:");DEBUG_PRINT(postConv);
    DEBUG_PRINT("  Smoothing:");DEBUG_PRINT(postSmooth);
    DEBUG_PRINT("  Mapping:");DEBUG_PRINT(postMap);
    DEBUG_PRINT("  Assignment:");DEBUG_PRINT(postAssign);
  #endif

  if(drone->STATUS.FAILSAFE == RC)Serial.print("FAILSAFE - RC ERROR\n");
}
