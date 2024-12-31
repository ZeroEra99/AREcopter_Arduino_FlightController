#ifndef ESC_H
#define ESC_H

#include <Servo.h>
#include "DataStructures.h"

class ESC
{
private:
  Servo servo; // Servo per l'ESC
  int pin;     // Pin specifico dell'attuatore
  int pwm_min; // PWM minimo
  int pwm_max; // PWM massimo

public:
  ESC(int pin, int min, int max); // Costruttore che accetta il pin
  void setup();                   // Implementazione del setup dell'ESC
  void write(int value);          // Implementazione del set dell'output PWM
};

#endif // ESC_H
