#ifndef RECEIVER_H
#define RECEIVER_H

#include "Drone.h"    // Include la struttura o classe Drone

// Funzioni per la gestione del ricevitore
void setupRC(Drone *drone);        // Configurazione iniziale del ricevitore
void readRC(Drone *drone); // Lettura e interpretazione dei segnali RC

#endif // RECEIVER_H
