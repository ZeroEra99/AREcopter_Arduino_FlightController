#ifndef DRONE_H
#define DRONE_H

#include <Arduino.h>  // Include le definizioni standard di Arduino
#include <Servo.h> // Per l'uso dei pin PWM con le ESC

// Struttura per i valori di input di controllo
struct ControlInput {
  float THR;  // Throttle
  float ROL;  // Roll
  float PIT;  // Pitch
  float HEAD; // Heading/Yaw
};

// Struttura per i valori di input IMU
struct IMUInput {
  float ROL;  // Roll
  float PIT;  // Pitch
  float HEAD; // Heading/Yaw
  float ROL_offset; // Offset Roll
  float PIT_offset; // Offset Pitch
  float HEAD_offset; // Offset Heading
};

// Struttura per i segnali inviati alle ESC
struct ESC_output {
  int FRL;  // Motore anteriore sinistro
  int FRR;  // Motore anteriore destro
  int BKL;  // Motore posteriore sinistro
  int BKR;  // Motore posteriore destro
};

// Struttura per i pin delle ESC
struct ESC_pins {
  Servo FRL;  // Motore anteriore sinistro
  Servo FRR;  // Motore anteriore destro
  Servo BKL;  // Motore posteriore sinistro
  Servo BKR;  // Motore posteriore destro
};

// Struttura per rappresentare i LED
struct LedLight {
  int Pin;       // Pin del LED
  bool Status;   // Stato del LED (acceso/spento)
};

// Enumerazione per gli stati di failsafe
enum FailSafe {
  NONE,   // Nessun problema
  RC,     // Problemi con il segnale RC
  IMU,    // Problemi con la IMU
  HAZARD  // Problema di sicurezza (es. roll/pitch eccessivo)
};

// Struttura per lo stato del drone
struct DroneStatus {
  bool isArmed;      // Stato armato/disarmato
  bool isStarting;   // Stato setup
  FailSafe FAILSAFE; // Stato failsafe attivo
};

// Struttura principale del drone
struct Drone {
  ControlInput RC_INPUT;      // Input dal radiocomando
  IMUInput IMU_INPUT;    // Input dalla IMU

  ESC_pins ESC;            // Oggetto contenente i pin delle ESC
  ESC_output ESC_OUTPUT;   // Oggetto contenente i segnali da inviare alle ESC

  LedLight redLed;       // LED rosso
  LedLight greenLed;     // LED verde

  DroneStatus STATUS;    // Stato generale del drone
};


#endif // DRONE_H
