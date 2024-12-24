#ifndef DEBUG_H
#define DEBUG_H

// Abilitare o disabilitare il debug globale
#define DEBUG_ENABLED 1  // Imposta 0 per disabilitare

// DEBUG Serial: Attiva/Disattiva stampa su Serial Monitor (Testo)
#define RC_DEBUG     0  // Imposta 0 per disabilitare
#define ESC_DEBUG    0  // Imposta 0 per disabilitare
#define IMU_DEBUG    0  // Imposta 0 per disabilitare
#define PID_DEBUG    0  // Imposta 0 per disabilitare
// DEBUG Graph: Attiva/Disattiva stampa per Serial Plotter (Grafico)
#define RC_GRAPH_DEBUG     0  // Imposta 0 per disabilitare
#define ESC_GRAPH_DEBUG    0  // Imposta 0 per disabilitare
#define IMU_GRAPH_DEBUG    0  // Imposta 0 per disabilitare
#define PID_GRAPH_DEBUG    0  // Imposta 0 per disabilitare

#if DEBUG_ENABLED
    #define DEBUG_PRINT(msg)  Serial.print(msg)
    #define DEBUG_PRINTLN(msg) Serial.println(msg)
#else
    #define DEBUG_PRINT(msg)  // Non fare nulla
    #define DEBUG_PRINTLN(msg) // Non fare nulla
#endif

#endif // DEBUG_H
