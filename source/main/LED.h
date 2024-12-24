#ifndef LED_H
#define LED_H

#include "drone.h"

// Dichiarazioni delle funzioni per il controllo dei LED
void onLED(LedLight *led);
void offLED(LedLight *led);
void writeLED(Drone *drone);
void setupLED(Drone *drone);

#endif // LED_H
