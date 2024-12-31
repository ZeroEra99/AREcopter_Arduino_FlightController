#ifndef DRONE_H
#define DRONE_H

#include "IA6B.h"
#include "ESC.h"
#include "BNO055.h"
#include "led.h"
#include "PIDcontrol.h"

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

  // Controllo del volo con PID
  PIDControl pidPitchAngle = PIDControl(KP_ROLL_ANGLE, KI_ROLL_ANGLE, KD_ROLL_ANGLE, MAX_INTEGRAL);
  PIDControl pidRollAngle = PIDControl(KP_ROLL_ANGLE, KI_ROLL_ANGLE, KD_ROLL_ANGLE, MAX_INTEGRAL);
  PIDControl pidYawAngle = PIDControl(KP_YAW_ANGLE, KI_YAW_ANGLE, KD_YAW_ANGLE, MAX_INTEGRAL);
  PIDControl pidPitchGyro = PIDControl(KP_ROLL_GYRO, KI_ROLL_GYRO, KD_ROLL_GYRO, MAX_INTEGRAL);
  PIDControl pidRollGyro = PIDControl(KP_ROLL_GYRO, KI_ROLL_GYRO, KD_ROLL_GYRO, MAX_INTEGRAL);
  PIDControl pidYawGyro = PIDControl(KP_YAW_GYRO, KI_YAW_GYRO, KD_YAW_GYRO, MAX_INTEGRAL);

  FlightData pidOffset; // Offset PID (correzioni applicate ai dati)
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

  // Funzioni per il calcolo dei dati di volo e degli output
  void computeFlightData(); // Calcola gli offset PID in base ai dati di volo
  void computeOutput();     // Calcola gli output per i motori (applica gli offset PID)

  // Applica gli output ai motori (ESC)
  void updateMotors(); 
};

#endif // DRONE_H
