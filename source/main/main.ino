#include "ESC.h"
#include "LED.h"
#include "Receiver.h"
#include "IMU.h"
#include "Control.h"
#include "Drone.h"

Drone drone;            // Creazione di un'istanza del drone
long loop_timer;        // Timer per il loop
float frequency;
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

  // Setup del timer per il loop
  loop_timer = micros();
}

void loop() {
  // Leggi input (IMU e RC)
  readIMU(&drone);
  readRC(&drone);

  // Controller PID
  PID(&drone, 0.01); // Passa il valore fisso di 0.01 secondi (100Hz)

  // Scrivi output (LED e ESC)
  writeLED(&drone);
  writeESC(&drone);

  // Calcola il tempo impiegato per il loop
  long time_taken = micros() - loop_timer;

  // Aspetta per completare l'intervallo di 100 ms
  if (time_taken < 100000) {
    delayMicroseconds(100000 - time_taken);
  }

  // Aggiorna il timer
  loop_timer = micros();
}

