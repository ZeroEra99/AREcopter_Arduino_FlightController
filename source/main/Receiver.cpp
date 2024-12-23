#include "Receiver.h"
#include "Config.h"  // Include i parametri di configurazione (pin, costanti)

// Implementazione di setupRC
void setupRC() {
  Serial.print("RC setup starting.\n");

  pinMode(RC_INPUT_THROTTLE_PIN, INPUT);
  pinMode(RC_INPUT_ROLL_PIN, INPUT);
  pinMode(RC_INPUT_PITCH_PIN, INPUT);
  pinMode(RC_INPUT_HEAD_PIN, INPUT);

  Serial.print("RC setup completed.\n");
}

// Implementazione di readRC
void readRC(Drone *drone) {
  // Lettura dei valori RC
  drone->RC_INPUT.THR = pulseIn(RC_INPUT_THROTTLE_PIN, HIGH, 30000);
  drone->RC_INPUT.ROL = pulseIn(RC_INPUT_ROLL_PIN, HIGH, 30000);
  drone->RC_INPUT.PIT = pulseIn(RC_INPUT_PITCH_PIN, HIGH, 30000);
  drone->RC_INPUT.HEAD = pulseIn(RC_INPUT_HEAD_PIN, HIGH, 30000);

  // Controllo validità
  if (drone->RC_INPUT.THR == 0 || drone->RC_INPUT.ROL == 0 || drone->RC_INPUT.PIT == 0 || drone->RC_INPUT.HEAD == 0) {
    Serial.print("RC input is invalid.\n");
    drone->STATUS.FAILSAFE = RC;
  }

  // Arm e Disarm
  if (drone->RC_INPUT.THR < (IO_MIN + ARMING_RANGE) &&
      drone->RC_INPUT.HEAD < (IO_MIN + ARMING_RANGE) &&
      drone->RC_INPUT.PIT < (IO_MIN + ARMING_RANGE) &&
      drone->RC_INPUT.ROL > (IO_MAX - ARMING_RANGE)) {
    Serial.print("Drone status: armed.\n");
    drone->STATUS.isArmed = true;
  } else if (drone->RC_INPUT.THR < (IO_MIN + ARMING_RANGE) &&
             drone->RC_INPUT.PIT < (IO_MAX + ARMING_RANGE) &&
             drone->RC_INPUT.HEAD > (IO_MAX - ARMING_RANGE) &&
             drone->RC_INPUT.ROL < (IO_MIN + ARMING_RANGE)) {
    Serial.print("Drone status: unarmed\n");
    drone->STATUS.isArmed = false;
  }

  // Conversione e mappatura
  drone->RC_INPUT.ROL = constrain(mapfloat(drone->RC_INPUT.ROL, IO_MIN, IO_MAX, -MAX_RC_INPUT_ROLL, MAX_RC_INPUT_ROLL), -MAX_RC_INPUT_ROLL, MAX_RC_INPUT_ROLL);
  drone->RC_INPUT.PIT = constrain(mapfloat(drone->RC_INPUT.PIT, IO_MIN, IO_MAX, -MAX_RC_INPUT_PITCH, MAX_RC_INPUT_PITCH), -MAX_RC_INPUT_PITCH, MAX_RC_INPUT_PITCH);
  drone->RC_INPUT.HEAD = constrain(mapfloat(drone->RC_INPUT.HEAD, IO_MIN, IO_MAX, -MAX_RC_INPUT_HEAD, MAX_RC_INPUT_HEAD), -MAX_RC_INPUT_HEAD, MAX_RC_INPUT_HEAD);
  drone->RC_INPUT.THR = constrain(mapfloat(drone->RC_INPUT.THR, IO_MIN, IO_MAX, 0, MAX_RC_INPUT_THROTTLE), 0, MAX_RC_INPUT_THROTTLE);

  // Smoothing
  if (abs(drone->RC_INPUT.ROL) < MIN_RC_INPUT_ROLL) drone->RC_INPUT.ROL = 0;
  if (abs(drone->RC_INPUT.PIT) < MIN_RC_INPUT_PITCH) drone->RC_INPUT.PIT = 0;
  if (abs(drone->RC_INPUT.HEAD) < MIN_RC_INPUT_HEAD) drone->RC_INPUT.HEAD = 0;
  if (abs(drone->RC_INPUT.THR) < MIN_RC_INPUT_THROTTLE) drone->RC_INPUT.THR = 0;

  // Mappatura finale
  if (hypot(drone->RC_INPUT.ROL, drone->RC_INPUT.PIT) > hypot(MAX_RC_INPUT_ROLL, MAX_RC_INPUT_PITCH)) {
    drone->RC_INPUT.ROL *= MAX_RC_INPUT_ROLL / hypot(drone->RC_INPUT.ROL, drone->RC_INPUT.PIT);
    drone->RC_INPUT.PIT *= MAX_RC_INPUT_PITCH / hypot(drone->RC_INPUT.ROL, drone->RC_INPUT.PIT);
  }

  // Logging per debugging
  Serial.print("\n  RC ROLL -> ");
  Serial.print(drone->RC_INPUT.ROL);
  Serial.print("  RC PITCH -> ");
  Serial.print(drone->RC_INPUT.PIT);
  Serial.print("  RC HEAD -> ");
  Serial.print(drone->RC_INPUT.HEAD);
  Serial.print("  RC THROTTLE -> ");
  Serial.print(drone->RC_INPUT.THR);

  if (drone->STATUS.FAILSAFE == RC)
    Serial.print("FAILSAFE - RC ERROR\n");
}
