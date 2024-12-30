#include "IA6B.h"
#include "Utils.h"
#include "PWM.hpp"

bool ia6b_data_error = false;
PWM pwm_throttle;
PWM pwm_roll;
int last_pitch, last_yaw;
DigitalData digitalDataError = {INVALID, INVALID, INVALID, INVALID};

IA6B::IA6B(AnalogData pin, int pulse_min, int pulse_max) : pin(pin), pulse_min(pulse_min), pulse_max(pulse_max)
{
  calibration = false;
  cycleCounter = 0;
  raw_data = {0, 0, 0, 0};
  flightData = {0, 0, 0, 0};
  offset = {0, 0, 0, 0};
  pwm_throttle.attach(pin.throttle);
  pwm_roll.attach(pin.roll);
}

void IA6B::setup()
{
  // Setup per il ricevitore
  Serial.print("IA6B setup starting...\n");
  pinMode(pin.pitch, INPUT);
  pinMode(pin.yaw, INPUT);
  pwm_throttle.begin(1);
  pwm_roll.begin(1);
  while (!calibration)
  {
    calibration = calibrate();
  }

  Serial.print("IA6B setup complete\n");
}

bool IA6B::calibrate()
{
  Serial.print("Calibrating IA6B...\n");
  int raw_data_avg_pitch = 0;
  int raw_data_avg_yaw = 0;
  for (int i = 0; i < CALIBRATION_CYCLES; i++)
  {
    int raw_data_pitch = pulseIn(pin.pitch, HIGH, 30000);
    int raw_data_yaw = pulseIn(pin.yaw, HIGH, 30000);
    if (raw_data_pitch == 0 || raw_data_yaw == 0)
    {
      Serial.print("Calibration failed.\n");
      return false;
    }
    else
    {
      raw_data_avg_pitch += raw_data_pitch / CALIBRATION_CYCLES;
      raw_data_avg_yaw += raw_data_yaw / CALIBRATION_CYCLES;
    }
  }

  offset.pitch = (pulse_min + pulse_max) / 2 - raw_data_avg_pitch;
  offset.yaw = (pulse_min + pulse_max) / 2 - raw_data_avg_yaw;

  Serial.print("IA6B Calibration complete.\n");
  return true;
}

bool IA6B::read()
{
  // Leggi i valori dal ricevitore, se un valore non è compreso fra pulse_min e max, il valore è considerato non valido
  bool isValid[4] = {true, true, true, true};
  // Leggi sempre Throttle e Roll
  if ((raw_data.throttle = pwm_throttle.getValue()) == 0)
    isValid[THROTTLE] = false;
  if ((raw_data.roll = pwm_roll.getValue()) == 0)
    isValid[ROLL] = false;
  // Alterna tra la lettura di pitch e yaw ogni PULSEIN_READ_CYCLE cicli
  if (cycleCounter % PULSEIN_READ_CYCLE == 0)
  {
    if (readPitchNext)
    {
      // Leggi Pitch
      if ((raw_data.pitch = pulseIn(pin.pitch, HIGH, 30000)) == 0)
        isValid[PITCH] = false;
      readPitchNext = false; // La prossima lettura sarà yaw
    }
    else
    {
      // Leggi Yaw
      if ((raw_data.yaw = pulseIn(pin.yaw, HIGH, 30000)) == 0)
        isValid[YAW] = false;
      readPitchNext = true; // La prossima lettura sarà pitch
    }
  }

  // Se un valore non è valido, stampa un messaggio di errore indicando quale
  if (!isValid[THROTTLE] || !isValid[ROLL] || !isValid[PITCH] || !isValid[YAW])
  {
    Serial.print("\n  Invalid IA6B data ->");
    // Stampa i valori non validi
    if (!isValid[THROTTLE])
      Serial.print(" Throttle,");
    if (!isValid[ROLL])
      Serial.print(" Roll,");
    if (!isValid[PITCH])
      Serial.print(" Pitch,");
    if (!isValid[YAW])
      Serial.print(" Yaw");
    return false;
  }
  else
  {
    flightData.throttle = raw_data.throttle + offset.throttle;
    flightData.roll = raw_data.roll + offset.roll;
    flightData.pitch = raw_data.pitch + offset.pitch;
    flightData.yaw = raw_data.yaw + offset.yaw;

#if DEBUG_IA6B
    DEBUG_PRINT("  IA6B Data ->")
    DEBUG_PRINT(" Throttle: ");
    DEBUG_PRINT(flightData.throttle);
    DEBUG_PRINT(" Roll: ");
    DEBUG_PRINT(flightData.roll);
    DEBUG_PRINT(" Pitch: ");
    DEBUG_PRINT(flightData.pitch);
    DEBUG_PRINT(" Yaw: ");
    DEBUG_PRINT(flightData.yaw);
#endif
  }
  cycleCounter++;
  return true;
}

void IA6B::getSticksPosition()
{
  StickPosition leftStick = {false, false, false, false};
  StickPosition rightStick = {false, false, false, false};
  // Controlla se i valori sono compresi tra i valori di ARM_TOLERANCE
  if (isInRange(flightData.throttle, IA6B_PWM_MIN, IA6B_PWM_MIN + IA6B_PWM_MAX * ARM_TOLERANCE * 0.01))
    leftStick.bottom = true;
  if (isInRange(flightData.throttle, IA6B_PWM_MAX, IA6B_PWM_MAX - IA6B_PWM_MAX * ARM_TOLERANCE * 0.01))
    leftStick.top = true;
  if (isInRange(flightData.yaw, IA6B_PWM_MIN, IA6B_PWM_MIN + IA6B_PWM_MAX * ARM_TOLERANCE * 0.01))
    leftStick.left = true;
  if (isInRange(flightData.yaw, IA6B_PWM_MAX, IA6B_PWM_MAX - IA6B_PWM_MAX * ARM_TOLERANCE * 0.01))
    leftStick.right = true;
  if (isInRange(flightData.roll, IA6B_PWM_MIN, IA6B_PWM_MIN + IA6B_PWM_MAX * ARM_TOLERANCE * 0.01))
    rightStick.left = true;
  if (isInRange(flightData.roll, IA6B_PWM_MAX, IA6B_PWM_MAX - IA6B_PWM_MAX * ARM_TOLERANCE * 0.01))
    rightStick.right = true;
  if (isInRange(flightData.pitch, IA6B_PWM_MIN, IA6B_PWM_MIN + IA6B_PWM_MAX * ARM_TOLERANCE * 0.01))
    rightStick.bottom = true;
  if (isInRange(flightData.pitch, IA6B_PWM_MAX, IA6B_PWM_MAX - IA6B_PWM_MAX * ARM_TOLERANCE * 0.01))
    rightStick.top = true;

  controlData = {leftStick, rightStick};
}

PilotData IA6B::getData()
{
  if (read())
  {
    getSticksPosition();

    PilotData data;
    data.pilotFlightData = flightData;
    data.pilotControlData = controlData;
    return data;
  }
  else
  {
    PilotData errorData;
    errorData.pilotFlightData = digitalDataError;
    return errorData;
  }
}