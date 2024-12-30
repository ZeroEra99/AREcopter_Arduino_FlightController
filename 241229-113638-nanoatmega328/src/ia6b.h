#ifndef IA6B_H
#define IA6B_H

#include "DataStructures.h"

class IA6B
{
private:
    bool calibrate();

    bool read();
    void getSticksPosition();

    AnalogData pin;
    int pulse_min, pulse_max;
    AnalogData offset;
    

    AnalogData raw_data;
    DigitalData flightData;
    ControlData controlData;
    
    int cycleCounter;
    bool calibration;
    bool readPitchNext;

public:
    IA6B(AnalogData pin, int pulse_min, int pulse_max); // Costruttore che accetta i pin
    void setup();
    PilotData getData();
};

#endif // IA6B_H
