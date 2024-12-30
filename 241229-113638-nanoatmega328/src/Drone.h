#ifndef DRONE_H
#define DRONE_H

#include "IA6B.h"
#include "ESC.h"
#include "BNO055.h"
#include "led.h"

class Drone
{
private:
  // Oggetti per il ricevitore, i servomotori, l'ESC e la IMU
  ESC motors[NUM_ESCs]; // ESC
  IA6B receiver;        // Ricevitore
  BNO055 imu;           // IMU
  LED ledRed = LED(LED_PIN_RED, RED);
  LED ledGreen = LED(LED_PIN_GREEN, GREEN);

  SystemState state; // Stato del sistema
  FlightData angleData; // Dati di orientamento
  FlightData gyroData;  // Dati di rotazione
  PilotData pilotData;          // Dati del pilota
  ControlDataType pilotInput; // Input del pilota

public:
  // Costruttore
  Drone();

  // Metodi
  void setup(); // Avvia il sistema
  void start();
  void disarm();
  void failsafe();

  // Funzioni per ottenere i dati del pilota e del volo
  void getFlightData();
  void getPilotData();

  void evaluateFlightData();
  void evaluatePilotCommands();
  void evaluateState();

  void manageLEDs();
  void updateLEDs();
};

#endif // DRONE_H
