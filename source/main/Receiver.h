#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>  // Per funzioni di Arduino come pulseIn
#include "Drone.h"    // Include la struttura o classe Drone
#include "Utils.h"    // Include la funzione mapfloat
// Funzioni per la gestione del ricevitore
void setupRC();        // Configurazione iniziale del ricevitore
void readRC(Drone *drone); // Lettura e interpretazione dei segnali RC

#endif // RECEIVER_H
