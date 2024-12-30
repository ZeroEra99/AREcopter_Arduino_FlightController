#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include "CONFIG.h"

// Struttura per i dati analogici del ricevitore e attuatori (Throttle, Pitch, Roll, Yaw)
struct AnalogData
{
  int throttle; // PWM del motore
  int pitch;    // PWM del servo pitch
  int roll;     // PWM del servo roll
  int yaw;      // PWM del servo yaw
};

// Struttura per i dati digitali del ricevitore e attuatori (Throttle, Pitch, Roll, Yaw)
struct DigitalData
{
  double throttle; // Valore digitale del motore
  double pitch;    // Valore digitale del servo pitch
  double roll;     // Valore digitale del servo roll
  double yaw;      // Valore digitale del servo yaw
};

// Struttura per i dati di volo (Pitch, Roll, Yaw)
struct FlightData
{
  double pitch; // Dati di orientamento pitch
  double roll;  // Dati di orientamento roll
  double yaw;   // Dati di orientamento yaw
};

struct StickPosition
{
  bool top;
  bool bottom;
  bool left;
  bool right;
};

struct ControlData
{
  StickPosition leftStick;
  StickPosition rightStick;
};

struct PilotData
{
  DigitalData pilotFlightData;
  ControlData pilotControlData;
};

// Struttura per i parametri PID (kP, kI, kD)
struct PIDParameters
{
  double kP; // Coefficiente proporzionale
  double kI; // Coefficiente integrale
  double kD; // Coefficiente derivativo
};

// Enumerazione per i vari output (Throttle, Pitch, Roll, Yaw)
enum OutputChannel
{
  THROTTLE = 0,
  PITCH,
  ROLL,
  YAW
};

// Enumerazione per i vari output (Front Left, Front Right, Rear Left, Rear Right)
enum MotorChannel
{
  FRL = 0,
  FRR,
  RRL,
  RRR
};

// Enumerazione per gli stati del sistema
enum SystemState
{
  STARTING = -2,
  FAILSAFE = -1,
  DISARMED,
  ARMED
};

// Enumerazione per i dati di volo (angle,gyro)
enum FlightDataType
{
  ANGLE = 0,
  GYRO
};

enum ControlDataType
{
  ERROR = -INVALID,
  NONE = -1,
  STOP_INPUT = 0,
  START_INPUT = 1
};

// Enumerazione per i colori dei LED
enum LEDColor
{
  RED = 0,
  GREEN
};

#endif // DATASTRUCTURES_H
