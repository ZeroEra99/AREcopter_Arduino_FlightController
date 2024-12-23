#ifndef LED_H
#define LED_H

#include "drone.h"

#define RED_LED_PIN 11
#define GREEN_LED_PIN 12

// Dichiarazioni delle funzioni per il controllo dei LED
void onLED(LedLight *led);
void offLED(LedLight *led);
void writeLED(Drone *drone);
void setupLED(Drone *drone);

#endif // LED_H
