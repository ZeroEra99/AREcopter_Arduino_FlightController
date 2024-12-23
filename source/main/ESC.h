#ifndef ESC_H
#define ESC_H

#include "Config.h"
#include "drone.h"

// Dichiarazione delle funzioni per l'inizializzazione e la gestione delle ESC
void setupESC(Drone *drone);    // Funzione di setup delle ESC
void writeESC(Drone *drone);    // Funzione per inviare segnali alle ESC

#endif // ESC_H
