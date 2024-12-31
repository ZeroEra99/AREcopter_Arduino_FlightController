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
  state = DISARMED;
  angleData = {0, 0, 0};
  gyroData = {0, 0, 0};
  pilotData = {0, 0, 0, 0};
  pilotInput = NONE;

  escData = {0, 0, 0, 0};
}

// Avvia il sistema (potrebbe includere la calibrazione)
void Drone::setup()
{
  for (int i = 0; i < NUM_ESCs; i++)
  {
    motors[i].setup(); // Inizializza ogni ESC
  }
  receiver.setup();   // Inizializza il ricevitore
  imu.setup(); // Inizializza l'IMU
  ledRed.setup();     // Inizializza LED rosso
  ledGreen.setup();   // Inizializza LED verde
}

void Drone::start()
{
  delay(2000);      // Attende 2 secondi -> Andrà resta non bloccante
  state = STARTING; // Imposta lo stato su STARTING
}

void Drone::disarm()
{
  state = DISARMED; // Imposta lo stato su DISARMED
}

void Drone::failsafe()
{
  state = FAILSAFE; // Imposta lo stato su FAILSAFE in caso di errore
}

// Funzione per ottenere i dati di volo (IMU)
void Drone::getFlightData()
{
  angleData = imu.read(ANGLE); // Ottiene i dati di orientamento
  gyroData = imu.read(GYRO);   // Ottiene i dati del giroscopio
}

// Funzione per ottenere i dati del pilota (ricevitore)
void Drone::getPilotData()
{
  pilotData = receiver.getData(); // Ottiene i dati dal ricevitore
  if (pilotData.pilotFlightData.throttle == INVALID || pilotData.pilotFlightData.pitch == INVALID || pilotData.pilotFlightData.roll == INVALID || pilotData.pilotFlightData.yaw == INVALID)
  {
    failsafe(); // Attiva il failsafe se i dati sono invalidi
    return;
  }
  // Converte i dati del pilota in valori digitali
  pilotData.pilotFlightData.throttle = pwm_to_digital(pilotData.pilotFlightData.throttle, IA6B_PWM_MIN, IA6B_PWM_MAX, MIN_THROTTLE, MAX_THROTTLE);
  pilotData.pilotFlightData.pitch = pwm_to_digital(pilotData.pilotFlightData.pitch, IA6B_PWM_MIN, IA6B_PWM_MAX, MIN_ROLL, MAX_ROLL);
  pilotData.pilotFlightData.roll = pwm_to_digital(pilotData.pilotFlightData.roll, IA6B_PWM_MIN, IA6B_PWM_MAX, MIN_ROLL, MAX_ROLL);
  pilotData.pilotFlightData.yaw = pwm_to_digital(pilotData.pilotFlightData.yaw, IA6B_PWM_MIN, IA6B_PWM_MAX, MIN_YAW, MAX_YAW);

  // Valida i comandi del pilota
  if (pilotData.pilotControlData.leftStick.bottom && pilotData.pilotControlData.rightStick.bottom)
  {
    if (pilotData.pilotControlData.leftStick.left && pilotData.pilotControlData.rightStick.right)
    {
      pilotInput = START_INPUT; // Inizia il volo
    }
    else if (pilotData.pilotControlData.leftStick.right && pilotData.pilotControlData.rightStick.left)
    {
      pilotInput = STOP_INPUT; // Ferma il volo
    }
  }
  else
    pilotInput = NONE;

#if DEBUG_PILOT_DATA
  // Debug dei dati del pilota
  DEBUG_PRINT("  Pilot Data ->")
  DEBUG_PRINT(" Throttle: ");
  DEBUG_PRINT(pilotData.flightInput.throttle);
  DEBUG_PRINT(" Pitch: ");
  DEBUG_PRINT(pilotData.flightInput.pitch);
  DEBUG_PRINT(" Roll: ");
  DEBUG_PRINT(pilotData.flightInput.roll);
  DEBUG_PRINT(" Yaw: ");
  DEBUG_PRINT(pilotData.flightInput.yaw);
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
  // Valuta i dati di volo per attivare il failsafe se i valori sono fuori range
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
      disarm(); // Ferma il drone
    }
  case DISARMED:
    if (pilotInput == START_INPUT)
    {
      start(); // Avvia il drone
    }
    break;
  case FAILSAFE:
    if (pilotInput == START_INPUT)
    {
      delay(2000); // Attendi 2 secondi sennò al ciclo successivo passerà da disarmed a armed senza l'intenzione del pilota. Andrà introdotto un counter
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
    // Procedura di avvio
    delay(2000);   // Attendi 2 secondi -> Andrà resta non bloccante
    state = ARMED; // Passa allo stato ARMED
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
    ledRed.setState(true);   // LED rosso lampeggiante
    ledGreen.setState(true); // LED verde lampeggiante
    break;
  case FAILSAFE:
    ledRed.setState(MS_BLINK_ON / 2, MS_BLINK_OFF / 2); // LED rosso lampeggiante veloce
    ledGreen.setState(false);                           // LED verde spento
    break;
  case DISARMED:
    ledRed.setState(false);                           // LED rosso spento
    ledGreen.setState(MS_BLINK_ON / 3, MS_BLINK_OFF); // LED verde lampeggiante lento
    break;
  case ARMED:
    ledRed.setState(false);  // LED rosso spento
    ledGreen.setState(true); // LED verde acceso
    break;
  }
}

// Funzione per aggiornare i LED
void Drone::updateLEDs()
{
  ledRed.update();
  ledGreen.update();
}

// Funzione per calcolare i valori degli ESC in base ai dati di volo
void Drone::computeOutput()
{
  if (state != ARMED)
  {
    escData = {ESC_PWM_MIN, ESC_PWM_MIN, ESC_PWM_MIN, ESC_PWM_MIN};
    return;
  }

  FlightData pidOffset = flightController.computeData(angleData, gyroData, pilotData.pilotFlightData);

  ESCData escDataTemp;
  escDataTemp.frl = pilotData.pilotFlightData.throttle + (pidOffset.pitch - pidOffset.roll - pidOffset.yaw) * pilotData.pilotFlightData.throttle * 0.01;
  escDataTemp.frr = pilotData.pilotFlightData.throttle + (pidOffset.pitch + pidOffset.roll + pidOffset.yaw) * pilotData.pilotFlightData.throttle * 0.01;
  escDataTemp.rrl = pilotData.pilotFlightData.throttle + (-pidOffset.pitch - pidOffset.roll + pidOffset.yaw) * pilotData.pilotFlightData.throttle * 0.01;
  escDataTemp.rrr = pilotData.pilotFlightData.throttle + (-pidOffset.pitch + pidOffset.roll - pidOffset.yaw) * pilotData.pilotFlightData.throttle * 0.01;

  escData.frl = digital_to_pwm(escDataTemp.frl, MIN_THROTTLE, MAX_THROTTLE, ESC_PWM_MIN, ESC_PWM_MAX);
  escData.frr = digital_to_pwm(escDataTemp.frr, MIN_THROTTLE, MAX_THROTTLE, ESC_PWM_MIN, ESC_PWM_MAX);
  escData.rrl = digital_to_pwm(escDataTemp.rrl, MIN_THROTTLE, MAX_THROTTLE, ESC_PWM_MIN, ESC_PWM_MAX);
  escData.rrr = digital_to_pwm(escDataTemp.rrr, MIN_THROTTLE, MAX_THROTTLE, ESC_PWM_MIN, ESC_PWM_MAX);
}

// Funzione per aggiornare i motori
void Drone::updateMotors()
{
  int pwm[NUM_ESCs];
  pwm[0] = digital_to_pwm(escData.frl, MIN_THROTTLE, MAX_THROTTLE, ESC_PWM_MIN, ESC_PWM_MAX);
  pwm[1] = digital_to_pwm(escData.frr, MIN_THROTTLE, MAX_THROTTLE, ESC_PWM_MIN, ESC_PWM_MAX);
  pwm[2] = digital_to_pwm(escData.rrl, MIN_THROTTLE, MAX_THROTTLE, ESC_PWM_MIN, ESC_PWM_MAX);
  pwm[3] = digital_to_pwm(escData.rrr, MIN_THROTTLE, MAX_THROTTLE, ESC_PWM_MIN, ESC_PWM_MAX);

  // Invia i segnali PWM agli ESC
  for (int i = 0; i < NUM_ESCs; i++)
  {
    motors[i].write(pwm[i]);
  }
}
