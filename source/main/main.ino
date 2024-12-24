#include "ESC.h"
#include "LED.h"
#include "Receiver.h"
#include "IMU.h"
#include "Control.h"
#include "Drone.h"

Drone drone;            // Creazione di un'istanza del drone
long loop_timer;        // Timer per il loop

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
  setupRC();
  setupIMU(&drone);

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

  // Delay per mantenere la frequenza di loop costante
  while (micros() - loop_timer < 100000)
    ;  // Aspetta fino a 100ms (100Hz)
  loop_timer = micros(); // Reset del timer
}
