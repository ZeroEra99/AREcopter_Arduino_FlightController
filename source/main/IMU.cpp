#include "IMU.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Config.h"
#include "Debug.h"

// Configurazione della IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);  // Oggetto per il sensore BNO055
sensors_event_t event;  // Oggetto per memorizzare i dati del sensore

// Funzione di setup della IMU
void setupIMU(Drone *drone) {
  Serial.print("IMU setup starting.\n");

  // Inizializzazione del sensore
  if (!bno.begin()) {
    // Caso di errore
    Serial.print("IMU setup failed.\n");
    while (1)
      ; // Entra in loop infinito se l'inizializzazione fallisce
  }
  // Successo
  bno.setExtCrystalUse(true);

  delay(1000); // Aspetta 100 ms per la stabilizzazione della IMU

  // Acquisizione dell'orientamento iniziale per il calcolo dell'offset
  sensors_event_t initial_event;
  if (!bno.getEvent(&initial_event)) {
    Serial.print("IMU setup failed to read initial orientation.\n");
    while (1) ; // Se non riesce a leggere i dati iniziali, rimane in loop infinito
  }


  Serial.print(initial_event.orientation.roll);  Serial.print(initial_event.orientation.pitch);
  Serial.print("banana");

  // Calcolo dell'offset di montaggio, supponendo che il drone sia in piano
  drone->IMU_INPUT.ROL_offset = initial_event.orientation.heading;
  drone->IMU_INPUT.PIT_offset = initial_event.orientation.pitch;
  drone->IMU_INPUT.HEAD_offset = initial_event.orientation.roll;

  Serial.print("IMU setup completed.\n");
  Serial.print("Initial IMU offsets: ");
  Serial.print("ROL_offset: ");
  Serial.print(drone->IMU_INPUT.ROL_offset);
  Serial.print(", PIT_offset: ");
  Serial.print(drone->IMU_INPUT.PIT_offset);
  Serial.print(", HEAD_offset: ");
  Serial.println(drone->IMU_INPUT.HEAD_offset);
}

// Funzione per leggere i dati dalla IMU
void readIMU(Drone *drone) {
  // Legge l'orientamento dal sensore
  if (!bno.getEvent(&event)) {
    drone->STATUS.FAILSAFE = IMU;
    Serial.print("FAILSAFE - IMU ERROR.\n");
  }

  // Compensazione dell'errore di montaggio: applica l'offset calcolato
  drone->IMU_INPUT.ROL = event.orientation.heading - drone->IMU_INPUT.ROL_offset;
  drone->IMU_INPUT.PIT = event.orientation.pitch - drone->IMU_INPUT.PIT_offset;
  drone->IMU_INPUT.HEAD = event.orientation.roll - drone->IMU_INPUT.HEAD_offset;

  // Controllo di sicurezza: Roll e Pitch fuori dai limiti
  if (drone->IMU_INPUT.ROL > MAX_ROLL * 2 || drone->IMU_INPUT.ROL < -MAX_ROLL * 2) {
    drone->STATUS.FAILSAFE = HAZARD;
    Serial.print("FAILSAFE - EXCESSIVE ROLL.\n");
  }
  if (drone->IMU_INPUT.PIT > MAX_PITCH * 2 || drone->IMU_INPUT.PIT < -MAX_PITCH * 2) {
    drone->STATUS.FAILSAFE = HAZARD;
    Serial.print("FAILSAFE - EXCESSIVE PITCH.\n");
  }

  // Stampa i valori
  #if IMU_DEBUG
    DEBUG_PRINT("\nIMU Roll -> ");
    DEBUG_PRINT(drone->IMU_INPUT.ROL);
    DEBUG_PRINT("IMU Pitch -> ");
    DEBUG_PRINT(drone->IMU_INPUT.PIT);
    DEBUG_PRINT("IMU Heading -> ");
    DEBUG_PRINT(drone->IMU_INPUT.HEAD);
  #endif

  #if IMU_GRAPH_DEBUG
    // Debug grafico per IMU
    DEBUG_PRINT("IMU_Roll:"); DEBUG_PRINT(drone->IMU_INPUT.ROL);
    DEBUG_PRINT(",IMU_Pitch:"); DEBUG_PRINT(drone->IMU_INPUT.PIT);
    DEBUG_PRINT(",IMU_Yaw:"); DEBUG_PRINT(drone->IMU_INPUT.YAW);
    DEBUG_PRINTLN();
  #endif

  // Stampa messaggi di errore in caso di failsafe
  if (drone->STATUS.FAILSAFE == IMU)
    Serial.print("FAILSAFE - IMU ERROR\n");
  if (drone->STATUS.FAILSAFE == HAZARD)
    Serial.print("FAILSAFE - HAZARD\n");
}
