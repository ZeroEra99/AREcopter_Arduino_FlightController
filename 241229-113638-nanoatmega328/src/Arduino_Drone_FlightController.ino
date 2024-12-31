#include <Arduino.h>
#include "Drone.h"

Drone drone; // Dichiarazione dell'oggetto Drone

void setup()
{
    Serial.begin(9600);
    drone.setup(); // Avvia il sistema
}

void loop()
{
    DEBUG_PRINTLN();
    drone.getFlightData(); // Ottieni i dati di volo
    drone.getPilotData();  // Ottieni i dati del pilota

    drone.evaluateFlightData();
    drone.evaluatePilotCommands();
    drone.evaluateState();

    drone.manageLEDs();
    drone.updateLEDs(); // Aggiorna i LED

    drone.computeFlightData(); // Calcola i dati di volo
    drone.computeOutput();     // Calcola l'output

    drone.updateMotors(); // Aggiorna i motori
}
