#ifndef LED_H
#define LED_H

#include "DataStructures.h"

class LED
{
private:
    int pin;
    LEDColor color;

    bool state, blinkState;
    int time_on, time_off; // Pin specifico dell'attuatore

public:
    LED(int pin, LEDColor color);     // Costruttore che accetta il pin
    void setup();                     // Implementazione del setup dell'ESC
    void setState(bool desiredState); // Implementazione del set dell'output PWM
    void setState(int on, int off);   // Implementazione del set dell'output PWM
    void update();
};

#endif // LED_H