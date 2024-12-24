#include "ESC.h"
#include "Config.h"
#include "Debug.h"

// Funzione di setup delle ESC
void setupESC(Drone *drone) {
  Serial.print("ESC setup starting.\n");

  // Impostazione dei pin delle ESC
  drone->ESC.FRL.attach(ESC_OUTPUT_FRL_PIN);
  drone->ESC.FRR.attach(ESC_OUTPUT_FRR_PIN);
  drone->ESC.BKL.attach(ESC_OUTPUT_BKL_PIN);
  drone->ESC.BKR.attach(ESC_OUTPUT_BKR_PIN);

  // Impostazione iniziale delle ESC al valore minimo
  drone->ESC.FRL.writeMicroseconds(IO_MIN);
  drone->ESC.FRR.writeMicroseconds(IO_MIN);
  drone->ESC.BKL.writeMicroseconds(IO_MIN);
  drone->ESC.BKR.writeMicroseconds(IO_MIN);

  // Completamento della configurazione
  Serial.print("ESC setup completed.\n");
}


// Funzione per inviare i segnali alle ESC
void writeESC(Drone *drone) {
  drone->ESC.FRL.writeMicroseconds(drone->ESC_OUTPUT.FRL);  // Invia il segnale al motore anteriore sinistro
  drone->ESC.FRR.writeMicroseconds(drone->ESC_OUTPUT.FRR);  // Invia il segnale al motore anteriore destro
  drone->ESC.BKL.writeMicroseconds(drone->ESC_OUTPUT.BKL);  // Invia il segnale al motore posteriore sinistro
  drone->ESC.BKR.writeMicroseconds(drone->ESC_OUTPUT.BKR);  // Invia il segnale al motore posteriore destro

  // Stampa i valori
  #if ESC_DEBUG
    DEBUG_PRINT("  Front Left ESC -> ");
    DEBUG_PRINT(drone->ESC_OUTPUT.FRL);
    DEBUG_PRINT("  Front Right ESC -> ");
    DEBUG_PRINT(drone->ESC_OUTPUT.FRR);
    DEBUG_PRINT("  Back Left ESC -> ");
    DEBUG_PRINT(drone->ESC_OUTPUT.BKL);
    DEBUG_PRINT("  Back Right ESC -> ");
    
    DEBUG_PRINTLN(drone->ESC_OUTPUT.BKR);
  #endif

  #if ESC_GRAPH_DEBUG
    // Debug grafico per ESC
    DEBUG_PRINT("ESC_FrontLeft:"); DEBUG_PRINT(drone->ESC_OUTPUT.FRL);
    DEBUG_PRINT(",ESC_FrontRight:"); DEBUG_PRINT(drone->ESC_OUTPUT.FRR);
    DEBUG_PRINT(",ESC_BackLeft:"); DEBUG_PRINT(drone->ESC_OUTPUT.BKL);
    DEBUG_PRINT(",ESC_BackRight:"); DEBUG_PRINT(drone->ESC_OUTPUT.BKR);
    DEBUG_PRINTLN();
  #endif
}
