#include "Drone.h"
#include "Utils.h"
#include <Arduino.h>

static AnalogData ia6b_pin = {IA6B_PIN_THROTTLE, IA6B_PIN_PITCH, IA6B_PIN_ROLL, IA6B_PIN_YAW};

// Costruttore
Drone::Drone()
    : motors{
          ESC(ESC_PIN_FRL, ESC_PWM_MIN, ESC_PWM_MAX),
          ESC(ESC_PIN_FRR, ESC_PWM_MIN, ESC_PWM_MAX),
          ESC(ESC_PIN_RRL, ESC_PWM_MIN, ESC_PWM_MAX),
          ESC(ESC_PIN_RRR, ESC_PWM_MIN, ESC_PWM_MAX)},
      receiver(ia6b_pin, IA6B_PWM_MIN, IA6B_PWM_MAX), imu(), ledRed(LED_PIN_RED, RED), ledGreen(LED_PIN_GREEN, GREEN)
{
}

// Avvia il sistema (potrebbe includere la calibrazione)
void Drone::setup()
{
  state = DISARMED;

  for (int i = 0; i < NUM_ESCs; i++)
  {
    motors[i].setup();
  }
  receiver.setup();
  imu.bno055_setup();
  ledRed.setup();
  ledGreen.setup();
}

void Drone::start()
{
  delay(2000);
  state = STARTING;
}

void Drone::disarm()
{
  state = DISARMED;
}

void Drone::failsafe()
{
  state = FAILSAFE;
}

// Funzione per ottenere i dati di volo (IMU)
void Drone::getFlightData()
{
  angleData = imu.bno055_read(ANGLE);
  gyroData = imu.bno055_read(GYRO);
}

// Funzione per ottenere i dati del pilota (ricevitore)
void Drone::getPilotData()
{
  pilotData = receiver.getData();
  if (pilotData.pilotFlightData.throttle == INVALID || pilotData.pilotFlightData.pitch == INVALID || pilotData.pilotFlightData.roll == INVALID || pilotData.pilotFlightData.yaw == INVALID)
  {
    failsafe();
    return;
  }
  pilotData.pilotFlightData.throttle = pwm_to_digital(pilotData.pilotFlightData.throttle, IA6B_PWM_MIN, IA6B_PWM_MAX, MIN_THROTTLE, MAX_THROTTLE);
  pilotData.pilotFlightData.pitch = pwm_to_digital(pilotData.pilotFlightData.pitch, IA6B_PWM_MIN, IA6B_PWM_MAX, MIN_ROLL, MAX_ROLL);
  pilotData.pilotFlightData.roll = pwm_to_digital(pilotData.pilotFlightData.roll, IA6B_PWM_MIN, IA6B_PWM_MAX, MIN_ROLL, MAX_ROLL);
  pilotData.pilotFlightData.yaw = pwm_to_digital(pilotData.pilotFlightData.yaw, IA6B_PWM_MIN, IA6B_PWM_MAX, MIN_YAW, MAX_YAW);

  if (pilotData.pilotControlData.leftStick.bottom && pilotData.pilotControlData.rightStick.bottom)
  {
    if (pilotData.pilotControlData.leftStick.left && pilotData.pilotControlData.rightStick.right)
    {
      pilotInput = START_INPUT;
    }
    else if (pilotData.pilotControlData.leftStick.right && pilotData.pilotControlData.rightStick.left)
    {
      pilotInput = STOP_INPUT;
    }
  }else
    pilotInput = NONE;

#if DEBUG_PILOT_DATA
  DEBUG_PRINT("  Pilot Data ->")
  DEBUG_PRINT(" Throttle: ");
  DEBUG_PRINT(pilotData.pilotFlightData.throttle);
  DEBUG_PRINT(" Pitch: ");
  DEBUG_PRINT(pilotData.pilotFlightData.pitch);
  DEBUG_PRINT(" Roll: ");
  DEBUG_PRINT(pilotData.pilotFlightData.roll);
  DEBUG_PRINT(" Yaw: ");
  DEBUG_PRINT(pilotData.pilotFlightData.yaw);
  DEBUG_PRINT(" Control ->");
  DEBUG_PRINT(" Left Stick: ");
  DEBUG_PRINT(pilotData.pilotControlData.leftStick.bottom);
  DEBUG_PRINT(pilotData.pilotControlData.leftStick.top);
  DEBUG_PRINT(pilotData.pilotControlData.leftStick.left);
  DEBUG_PRINT(pilotData.pilotControlData.leftStick.right);
  DEBUG_PRINT(" Right Stick: ");
  DEBUG_PRINT(pilotData.pilotControlData.rightStick.bottom);
  DEBUG_PRINT(pilotData.pilotControlData.rightStick.top);
  DEBUG_PRINT(pilotData.pilotControlData.rightStick.left);
  DEBUG_PRINT(pilotData.pilotControlData.rightStick.right);

#endif
}

void Drone::evaluateFlightData()
{
  if (angleData.pitch > MAX_ROLL || angleData.pitch < MIN_ROLL || angleData.roll > MAX_ROLL || angleData.roll < MIN_ROLL)
  {
    failsafe();
    return;
  }
}

// Funzione per valutare i comandi del pilota
void Drone::evaluatePilotCommands()
{
  switch (state)
  {
  case STARTING:
    if (pilotInput == STOP_INPUT)
    {
      Serial.print("Abort\n");
      disarm();
    }
  case DISARMED:
    if (pilotInput == START_INPUT)
    {
      start();
    }
    break;
  case FAILSAFE:
    if (pilotInput == START_INPUT)
    {
      delay(2000);
      disarm();
      Serial.print("Clearing failsafe...\n");
    }
    break;
  case ARMED:
    if (pilotInput == STOP_INPUT)
    {
      Serial.print("Disarming...\n");
      disarm();
    }
    break;
  }
  return;
}

void Drone::evaluateState()
{
  switch (state)
  {
  case STARTING:
    // Starting procedure
    delay(2000);
    state = ARMED;
    break;
  case FAILSAFE:
    break;
  case DISARMED:
    break;
  default:
    break;
  }
}

// Funzione per aggiornare i LED
void Drone::manageLEDs()
{
  switch (state)
  {
  case STARTING:
    ledRed.setState(MS_BLINK_OFF, MS_BLINK_ON);
    ledGreen.setState(MS_BLINK_ON, MS_BLINK_OFF);
    break;
  case FAILSAFE:
    ledRed.setState(MS_BLINK_ON, MS_BLINK_OFF);
    ledGreen.setState(false);
    break;
  case DISARMED:
    ledRed.setState(false);
    ledGreen.setState(MS_BLINK_ON/3, MS_BLINK_OFF);
    break;
  case ARMED:
    ledRed.setState(false);
    ledGreen.setState(true);
    break;
  }
}

// Funzione per aggiornare i LED
void Drone::updateLEDs()
{
  ledRed.update();
  ledGreen.update();
}