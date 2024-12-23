#include "IMU.h"

// Configurazione della IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);  // Oggetto per il sensore BNO055
sensors_event_t event;  // Oggetto per memorizzare i dati del sensore

// Funzione di setup della IMU
void setupIMU() {
  Serial.print("IMU setup starting.\n");

  // Inizializzazione del sensore
  if (!bno.begin()) {
    // Caso di errore
    Serial.print("IMU setup failed.\n");
    while (1)
      ; // Entra in loop infinito se l'inizializzazione fallisce
  }
  // Successo
  bno.setExtCrystalUse(true);  // Usa il cristallo esterno per la precisione

  Serial.print("IMU setup completed.\n");
}

// Funzione per leggere i dati dalla IMU
void readIMU(Drone *drone) {
  // Legge l'orientamento dal sensore
  if (!bno.getEvent(&event)) {
    drone->STATUS.FAILSAFE = IMU;
    Serial.print("FAILSAFE - IMU ERROR.\n");
  }

  // Controllo di sicurezza: Roll e Pitch fuori dai limiti
  if (event.orientation.heading > MAX_ROLL * 2 || event.orientation.heading < -MAX_ROLL * 2) {
    drone->STATUS.FAILSAFE = HAZARD;
    Serial.print("FAILSAFE - EXCESSIVE ROLL.\n");
  }
  if (event.orientation.pitch > MAX_PITCH * 2 || event.orientation.pitch < -MAX_PITCH * 2) {
    drone->STATUS.FAILSAFE = HAZARD;
    Serial.print("FAILSAFE - EXCESSIVE PITCH.\n");
  }

  // Memorizza i dati dell'orientamento nella struttura del drone
  drone->IMU_INPUT.ROL = event.orientation.heading;
  drone->IMU_INPUT.PIT = event.orientation.pitch;
  drone->IMU_INPUT.HEAD = event.orientation.roll;

  // Stampa i valori (opzionale, per il debug)
  /*
  Serial.print("\nRoll ->  ");
  Serial.print(drone->IMU_INPUT.ROL);
  Serial.print("  Pitch -> ");
  Serial.print(drone->IMU_INPUT.PIT);
  */

  // Stampa messaggi di errore in caso di failsafe
  if (drone->STATUS.FAILSAFE == IMU)
    Serial.print("FAILSAFE - IMU ERROR\n");
  if (drone->STATUS.FAILSAFE == HAZARD)
    Serial.print("FAILSAFE - HAZARD\n");
}
