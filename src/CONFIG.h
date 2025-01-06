#ifndef CONFIG_H
#define CONFIG_H

// CONFIGURAZIONE HARDWARE

// PIN per il controllo dei segnali (Throttle, Pitch, Roll, Yaw, ESC, LED)
#define IA6B_PIN_THROTTLE 3
#define IA6B_PIN_PITCH 4
#define IA6B_PIN_ROLL 2
#define IA6B_PIN_YAW 8

#define ESC_PIN_FRL 5
#define ESC_PIN_FRR 6
#define ESC_PIN_RRL 9
#define ESC_PIN_RRR 10

#define LED_PIN_RED 11
#define LED_PIN_GREEN 12

// Intervalli di segnale PWM
#define ESC_PWM_MIN 1000
#define ESC_PWM_MAX 2000
#define IA6B_PWM_MIN 1000
#define IA6B_PWM_MAX 2000

// Numero di ESC
#define NUM_ESCs 4

// CONFIGURAZIONE SOFTWARE

// Parametri di lettura e calibrazione
#define PULSEIN_READ_CYCLE 5
#define CALIBRATION_CYCLES 100

// Valore costante per segnali non validi
#define INVALID -255

// Parametri per il lampeggio dei LED
#define MS_BLINK_ON 500
#define MS_BLINK_OFF 200

// CONFIGURAZIONE DEL FLIGHT CONTROLLER

// Limiti di input del pilota
#define MIN_THROTTLE 0.0
#define MAX_THROTTLE 100.0
#define MIN_ROLL -MAX_ROLL
#define MAX_ROLL 10.0
#define MIN_YAW -MAX_YAW
#define MAX_YAW 20.0
#define ARM_TOLERANCE 20.0

// Parametri PID per la stabilizzazione (giroscopio)
#define KP_ROLL_GYRO 0.0
#define KI_ROLL_GYRO 0.0
#define KD_ROLL_GYRO 0.0
#define KP_YAW_GYRO 0.0
#define KI_YAW_GYRO 0.0
#define KD_YAW_GYRO 0.0

// Parametri PID per la stabilizzazione (orientamento)
#define KP_ROLL_ANGLE 0.0
#define KI_ROLL_ANGLE 0.0
#define KD_ROLL_ANGLE 0.0
#define KP_YAW_ANGLE 0.0
#define KI_YAW_ANGLE 0.0
#define KD_YAW_ANGLE 0.0

// Limite massimo del termine integrale del PID
#define MAX_INTEGRAL 100.0

// CONFIGURAZIONE DEBUG

// Abilitazione globale per il debugging
#define DEBUG_ENABLED 0

// Debugging per componenti specifiche
#define DEBUG_IA6B 0
#define DEBUG_PILOT_DATA 1
#define DEBUG_FLIGHT_DATA_GYRO 0
#define DEBUG_FLIGHT_DATA_ANGLE 0
#define DEBUG_ESC 0
#define DEBUG_LED 0
#define DEBUG_BNO055 0

// Macro per la stampa di debug
#if DEBUG_ENABLED
#define DEBUG_PRINT(x) Serial.print(x);
#define DEBUG_PRINTLN(x) Serial.println(x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#endif // CONFIG_H
