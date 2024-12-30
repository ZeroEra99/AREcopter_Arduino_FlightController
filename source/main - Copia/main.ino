#include "ESC.h"
#include "LED.h"
#include "Receiver.h"
#include "IMU.h"
#include "Control.h"
#include "Drone.h"
#include "Debug.h"

Drone drone;            // Creazione di un'istanza del drone
void setup() {
  // Serial
  Serial.begin(9600);
  Serial.print("Drone setup starting.\n");

  // Stato iniziale
  drone.STATUS.isStarting = true;
  drone.STATUS.isArmed = false;
  drone.STATUS.FAILSAFE = NONE;

  // Setup Hardware
  setupESC(&drone);
  setupLED(&drone);
  setupIMU(&drone);
  setupRC(&drone);

  // Stato iniziale: completato setup
  drone.STATUS.isStarting = false;
  Serial.print("Drone setup completed.\n");
}

void loop() {
  int preTot = millis();
  DEBUG_PRINT("T (millis()):");DEBUG_PRINT(preTot);
  // Leggi input (IMU e RC)
  int preIMU = millis();
  readIMU(&drone);
  int postIMU = millis() - preIMU;
  
  int preRC = millis();
  readRC(&drone);
  int postRC = millis() - preRC;

  // Controller PID
  int prePID = millis();
  PID(&drone);
  int postPID = millis() - prePID;

  // Scrivi output (LED e ESC)
  int preLED = millis();
  writeLED(&drone);
  int postLED = millis() - preLED;

  int preESC = millis();
  writeESC(&drone);
  int postESC = millis() - preESC;


  int TOT = millis() - preTot;
  #if LOOP_EXTIME_DEBUG
    DEBUG_PRINT("  IMU:");DEBUG_PRINT(postIMU);
    DEBUG_PRINT("  RC:");DEBUG_PRINT(postRC);
    DEBUG_PRINT("  PID:");DEBUG_PRINT(postPID);
    DEBUG_PRINT("  LED:");DEBUG_PRINT(postLED);
    DEBUG_PRINT("  ESC:");DEBUG_PRINT(postESC);
    DEBUG_PRINT("  TOT:");DEBUG_PRINT(TOT);
  #endif

  DEBUG_PRINTLN();
}

