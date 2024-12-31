#ifndef DRONE_H
#define DRONE_H

#include "IA6B.h"
#include "ESC.h"
#include "BNO055.h"
#include "led.h"
#include "FlightController.h"

// Costruttore della classe Drone
class Drone
{
private:
  // Oggetti per il ricevitore, i motori (ESC), la IMU e i LED
  ESC motors[NUM_ESCs]; // Array di ESC per i motori
  IA6B receiver;
  BNO055 imu; // Unità di misura inerziale (IMU)
  LED ledRed = LED(LED_PIN_RED, RED);
  LED ledGreen = LED(LED_PIN_GREEN, GREEN);

  // Stato del sistema e dati di volo
  SystemState state;          // Stato corrente del sistema (ARMATO, DISARMATO, ecc.)
  FlightData angleData;       // Dati angolari di orientamento
  FlightData gyroData;        // Dati del giroscopio (velocità angolare)
  PilotData pilotData;        // Dati del pilota (input del pilota)
  ControlDataType pilotInput; // Comando del pilota (START, STOP, ecc.)

  // Oggetto per il controllo del volo
  FlightController flightController; // Controller di volo

  // Controllo del volo con PID
  ESCData escData;      // Output degli ESC (per i vari motori)

public:
  // Costruttore
  Drone();

  // Metodi principali
  void setup();    // Inizializza il sistema (setup hardware)
  void start();    // Avvia il drone (passaggio allo stato AVVIO)
  void disarm();   // Disarma il drone (disattiva i motori)
  void failsafe(); // Attiva il failsafe in caso di malfunzionamento

  // Funzioni per ottenere i dati di volo e i comandi del pilota
  void getFlightData();
  void getPilotData();

  // Funzioni per la gestione dei dati di volo e dei comandi del pilota
  void evaluateFlightData();    // Valuta la lavidità dei dati di volo
  void evaluatePilotCommands(); // Interpreta i comandi del pilota
  void evaluateState();         // Valuta lo stato attuale del sistema (ARMATO, FAILSAFE, ecc.)

  // Funzioni per gestire i LED
  void manageLEDs(); // Gestisce i LED in base allo stato del sistema
  void updateLEDs(); // Aggiorna lo stato dei LED

  // Funzioni per calcolare i valori degli ESC in base ai dati di volo
  void computeOutput(); 

  // Applica gli output ai motori (ESC)
  void updateMotors(); 
};

#endif // DRONE_H
