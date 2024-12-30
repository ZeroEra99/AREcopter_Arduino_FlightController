#include "IMU.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Config.h"
#include "Debug.h"

// Configurazione della IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);  // Oggetto per il sensore BNO055

void offsetCalibrationIMU(Drone *drone) {
  // Acquisizione dell'orientamento per il calcolo dell'offset
  sensors_event_t calibration_event;
  if (!bno.getEvent(&calibration_event)) {
    Serial.print("IMU offset calibration failed.\n");
    while (1)
      ;  // Se non riesce a leggere i dati iniziali, rimane in loop infinito
  }
  // Calcolo dell'offset di montaggio, supponendo che il drone sia in piano
  drone->IMU_INPUT.ROLL_offset = calibration_event.orientation.heading;
  drone->IMU_INPUT.PITCH_offset = calibration_event.orientation.pitch;
  drone->IMU_INPUT.HEAD_offset = calibration_event.orientation.roll;
  drone->STATUS.imuCalibrated = true;

  Serial.print(" IMU offsets -> ");
  Serial.print("ROLL_offset: ");
  Serial.print(drone->IMU_INPUT.ROLL_offset);
  Serial.print(", PITCH_offset: ");
  Serial.print(drone->IMU_INPUT.PITCH_offset);
  Serial.print(", HEAD_offset: ");
  Serial.println(drone->IMU_INPUT.HEAD_offset);
}

// Funzione di setup della IMU
void setupIMU(Drone *drone) {
  Serial.print("IMU setup starting.\n");

  // Inizializzazione del sensore
  if (!bno.begin()) {
    // Caso di errore
    Serial.print("IMU setup failed.\n");
    while (1)
      ;  // Entra in loop infinito se l'inizializzazione fallisce
  }
  drone->STATUS.imuCalibrated = false;
  // Successo
  bno.setExtCrystalUse(true);

  delay(1000);  // Aspetta 100 ms per la stabilizzazione della IMU
  Serial.print("IMU setup completed.\n");
}

// Funzione per leggere i dati dalla IMU
void readIMU(Drone *drone) {
  if(!drone->STATUS.imuCalibrated)offsetCalibrationIMU(drone);
  // Legge l'orientamento dal sensore
  sensors_event_t orientationData, angVelocityData;
  if (!bno.getEvent(&orientationData)) {
    drone->STATUS.FAILSAFE = IMU;
    if(drone->STATUS.imuCalibrated)drone->STATUS.imuCalibrated=false;
    Serial.print("FAILSAFE - IMU ERROR.\n");
  }
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Leggi orientamento con compensazione dell'errore di montaggio
  drone->IMU_INPUT.ROLL = orientationData.orientation.heading - drone->IMU_INPUT.ROLL_offset;
  drone->IMU_INPUT.PITCH = orientationData.orientation.pitch - drone->IMU_INPUT.PITCH_offset;
  drone->IMU_INPUT.HEAD = orientationData.orientation.roll - drone->IMU_INPUT.HEAD_offset;
  drone->IMU_INPUT.ROLL_RATE = -angVelocityData.gyro.z;
  drone->IMU_INPUT.PITCH_RATE = angVelocityData.gyro.y;
  drone->IMU_INPUT.HEAD_RATE = angVelocityData.gyro.x;

  // Controllo di sicurezza: Roll e Pitch fuori dai limiti
  if (drone->IMU_INPUT.ROLL > 90 || drone->IMU_INPUT.ROLL < -90 || drone->IMU_INPUT.PITCH > 60 || drone->IMU_INPUT.PITCH < -60) {
    drone->STATUS.FAILSAFE = HAZARD;
    drone->STATUS.isArmed = false;
    if(drone->STATUS.imuCalibrated)drone->STATUS.imuCalibrated=false;
    Serial.print("FAILSAFE - EXCESSIVE ANGLE.");
  }

// Stampa i valori
#if IMU_DEBUG
  /*
  DEBUG_PRINT("IMU Roll -> ");
  DEBUG_PRINT(drone->IMU_INPUT.ROLL);
  DEBUG_PRINT("IMU Pitch -> ");
  DEBUG_PRINT(drone->IMU_INPUT.PITCH);
  DEBUG_PRINT("IMU Heading -> ");
  DEBUG_PRINT(drone->IMU_INPUT.HEAD);
  */
  DEBUG_PRINT("IMU Roll Rate -> ");
  DEBUG_PRINT(drone->IMU_INPUT.ROLL_RATE);
  DEBUG_PRINT("IMU Pitch Rate -> ");
  DEBUG_PRINT(drone->IMU_INPUT.PITCH_RATE);
  DEBUG_PRINT("IMU Head Rate -> ");
  DEBUG_PRINT(drone->IMU_INPUT.HEAD_RATE);
#endif

#if IMU_GRAPH_DEBUG
  // Debug grafico per IMU
  DEBUG_PRINT("IMU_Roll:");
  DEBUG_PRINT(drone->IMU_INPUT.ROLL);
  DEBUG_PRINT(",IMU_Pitch:");
  DEBUG_PRINT(drone->IMU_INPUT.PITCH);
  DEBUG_PRINT(",IMU_Head:");
  DEBUG_PRINT(drone->IMU_INPUT.HEAD);
  DEBUG_PRINT(",IMU_RollRate:");
  DEBUG_PRINT(drone->IMU_INPUT.ROLL_RATE);
  DEBUG_PRINT(",IMU_PitchRate:");
  DEBUG_PRINT(drone->IMU_INPUT.PITCH_RATE);
  DEBUG_PRINT(",IMU_HeadRate:");
  DEBUG_PRINT(drone->IMU_INPUT.HEAD_RATE);
  DEBUG_PRINTLN();
#endif
}
