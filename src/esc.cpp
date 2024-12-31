#include "ESC.h"
#include "Utils.h"
#include <Arduino.h>

ESC::ESC(int pwm_pin, int pwm_min, int pwm_max) : pin(pwm_pin), pwm_min(pwm_min), pwm_max(pwm_max)
{
}

void ESC::setup()
{
    // Implementazione del setup dell'ESC (es. configurazione del pin PWM)
    Serial.print("ESC ");
    Serial.print(pin);
    Serial.print(" setup starting...\n");
    servo.attach(pin);
    servo.writeMicroseconds(pwm_min);
    Serial.print("ESC ");
    Serial.print(pin);
    Serial.print(" setup complete\n");
}

void ESC::write(int value)
{
    // Implementazione del set dell'output PWM dell'ESC
    if (!isInRange(value, pwm_min, pwm_max))
    {
        Serial.print("Invalid ESC value\n");
        return;
    }
    servo.writeMicroseconds(value);
}