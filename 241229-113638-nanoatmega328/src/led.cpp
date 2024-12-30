#include "led.h"
#include <Arduino.h>

LED::LED(int pin, LEDColor color) : pin(pin), color(color), state(false), blinkState(false), time_on(0), time_off(0)
{
    Serial.print("LED ");
    Serial.print(color);
    Serial.print(" created\n");
}

void LED::setup()
{
    Serial.print("LED ");
    Serial.print(color);
    Serial.print(" setup starting...\n");

    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delay(500);
    digitalWrite(pin, LOW);

    Serial.print("LED ");
    Serial.print(color);
    Serial.print(" setup complete\n");
}

void LED::setState(bool desiredState)
{
    blinkState = false;
    state != desiredState ? state = desiredState : state = state;
}

void LED::setState(int on, int off)
{
    blinkState = true;
    time_on = on;
    time_off = off;
}

void LED::update()
{
    static unsigned long lastUpdate = 0; // Memorizza l'ultimo momento in cui il LED è stato aggiornato
    static bool isOn = false;            // Stato corrente del LED durante il lampeggio

    if (!blinkState)
    {
        // Se il lampeggio non è attivo, mantieni lo stato corrente
        state ? digitalWrite(pin, HIGH) : digitalWrite(pin, LOW);
        return;
    }

    // Lampeggio non bloccante
    unsigned long currentMillis = millis();
    unsigned long millisPassed = currentMillis - lastUpdate;
    if (isOn && millisPassed >= (unsigned long)time_on)
    {
        // Se il LED è acceso e il tempo "on" è trascorso
        digitalWrite(pin, LOW);
        isOn = false;
        lastUpdate = currentMillis;
    }
    else if (!isOn && millisPassed >= (unsigned long)time_off)
    {
        // Se il LED è spento e il tempo "off" è trascorso
        digitalWrite(pin, HIGH);
        isOn = true;
        lastUpdate = currentMillis;
    }
}
