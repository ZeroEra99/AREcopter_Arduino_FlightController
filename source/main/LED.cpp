#include "LED.h"

// Funzione per accendere il LED
void onLED(LedLight *led) {
  digitalWrite(led->Pin, HIGH);
  led->Status = true;
}

// Funzione per spegnere il LED
void offLED(LedLight *led) {
  digitalWrite(led->Pin, LOW);
  led->Status = false;
}

// Funzione per scrivere lo stato dei LED in base allo stato del drone
void writeLED(Drone *drone) {
  offLED(&drone->redLed);   // Spegni il LED rosso
  offLED(&drone->greenLed); // Spegni il LED verde

  if (drone->STATUS.isStarting) {
    onLED(&drone->greenLed); // Accendi il LED verde
    onLED(&drone->redLed);   // Accendi il LED rosso
  } else if (drone->STATUS.isArmed) {
    onLED(&drone->greenLed); // Accendi solo il LED verde
  } else if (drone->STATUS.FAILSAFE || !drone->STATUS.isArmed) {
    onLED(&drone->redLed);   // Accendi solo il LED rosso
  }
}

// Funzione per configurare i LED (imposta i pin e stato iniziale)
void setupLED(Drone *drone) {
  Serial.print("LED setup starting\n");

  // Configura i pin dei LED
  drone->redLed.Pin = RED_LED_PIN;
  drone->greenLed.Pin = GREEN_LED_PIN;
  drone->redLed.Status = false;
  drone->greenLed.Status = false;

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  // Accende il LED rosso inizialmente
  onLED(&drone->redLed);
  onLED(&drone->redLed);  // Accensione duplicata per fare il reset

  // Scrivi lo stato finale dei LED
  writeLED(drone);

  Serial.print("LED setup completed.\n");
}
