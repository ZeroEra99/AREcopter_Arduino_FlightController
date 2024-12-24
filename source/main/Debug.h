#ifndef DEBUG_H
#define DEBUG_H

// Abilitare o disabilitare il debug globale
#define DEBUG_ENABLED 1  

// DEBUG Serial: Attiva/Disattiva stampa su Serial Monitor (Testo)
#define RC_DEBUG     0  
#define ESC_DEBUG    1  
#define IMU_DEBUG    0  
#define PID_DEBUG    0  
// DEBUG Graph: Attiva/Disattiva stampa per Serial Plotter (Grafico)
#define RC_GRAPH_DEBUG     0  
#define ESC_GRAPH_DEBUG    0  
#define IMU_GRAPH_DEBUG    0  
#define PID_GRAPH_DEBUG    0  

#if DEBUG_ENABLED
    #define DEBUG_PRINT(msg)  Serial.print(msg)
    #define DEBUG_PRINTLN(msg) Serial.println(msg)
#else
    #define DEBUG_PRINT(msg)  // Non fare nulla
    #define DEBUG_PRINTLN(msg) // Non fare nulla
#endif

#endif // DEBUG_H
