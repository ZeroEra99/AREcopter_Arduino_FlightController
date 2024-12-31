#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include "CONFIG.h"

// Struttura per i dati analogici del ricevitore e attuatori
struct AnalogData
{
  int throttle; // PWM del motore
  int pitch;    // PWM del servo pitch
  int roll;     // PWM del servo roll
  int yaw;      // PWM del servo yaw
};

// Struttura per i dati di orientamento del volo
struct FlightData
{
  double pitch; // Angolo pitch
  double roll;  // Angolo roll
  double yaw;   // Angolo yaw
};

// Struttura per i dati digitali del ricevitore e attuatori
struct flightInput
{
  double throttle; // Valore digitale del motore
  double pitch;    // Valore digitale del servo pitch
  double roll;     // Valore digitale del servo roll
  double yaw;      // Valore digitale del servo yaw
};

// Posizione degli stick (top, bottom, left, right)
struct StickPosition
{
  bool top;
  bool bottom;
  bool left;
  bool right;
};

// Struttura per i dati di controllo degli stick
struct ControlData
{
  StickPosition leftStick;
  StickPosition rightStick;
};

// Dati del pilota: input di volo e dati di controllo
struct PilotData
{
  flightInput pilotFlightData;
  ControlData pilotControlData;
};

// Parametri PID per i controlli
struct PIDParameters
{
  double kP; // Coefficiente proporzionale
  double kI; // Coefficiente integrale
  double kD; // Coefficiente derivativo
};

// Dati degli ESC
struct ESCData
{
  float frl; // Front Left ESC
  float frr; // Front Right ESC
  float rrl; // Rear Left ESC
  float rrr; // Rear Right ESC
};

// Canali di input (Throttle, Pitch, Roll, Yaw)
enum InputChannel
{
  THROTTLE = 0,
  PITCH,
  ROLL,
  YAW
};

// Canali dei motori (Front Left, Front Right, Rear Left, Rear Right)
enum MotorChannel
{
  FRL = 0,
  FRR,
  RRL,
  RRR
};

// Stati del sistema
enum SystemState
{
  STARTING = -2,
  FAILSAFE = -1,
  DISARMED,
  ARMED
};

// Tipi di dati di volo (angolo, giroscopio)
enum FlightDataType
{
  ANGLE = 0,
  GYRO
};

// Tipi di dati di controllo
enum ControlDataType
{
  ERROR = -INVALID,
  NONE = -1,
  STOP_INPUT = 0,
  START_INPUT = 1
};

// Colori dei LED
enum LEDColor
{
  RED = 0,
  GREEN
};

#endif // DATASTRUCTURES_H
