#ifndef ESC_H
#define ESC_H

#include <Servo.h>
#include "DataStructures.h"

class ESC
{
private:
  Servo servo; // Servo per l'ESC
  int pin;     // Pin specifico dell'attuatore
  int pwm_min;
  int pwm_max;

  DigitalData digitalOutput; // Output digitale (double)
  AnalogData analogOutput;   // Output PWM (analogico)

public:
  ESC(int pwm_pin, int pwm_min, int pwm_max); // Costruttore che accetta il pin
  void setup();                           // Implementazione del setup dell'ESC
  void write(int value);            // Implementazione del set dell'output PWM
};

#endif // ESC_H
