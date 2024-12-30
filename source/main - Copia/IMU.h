#ifndef IMU_H
#define IMU_H

#include "drone.h"  // Include la struttura Drone

// Definizioni delle costanti
#define MAX_PITCH 14
#define MAX_ROLL 12

// Funzioni per l'inizializzazione e la lettura della IMU
void setupIMU(Drone *drone);              // Configura la IMU
void readIMU(Drone *drone);   // Legge i dati dalla IMU e aggiorna lo stato del drone

#endif // IMU_H
