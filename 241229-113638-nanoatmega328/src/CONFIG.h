#ifndef CONFIG_H
#define CONFIG_H

/* Parametri HARDWARE */
/* PIN */
#define IA6B_PIN_THROTTLE 3
#define IA6B_PIN_PITCH 4
#define IA6B_PIN_ROLL 2
#define IA6B_PIN_YAW 8
#define ESC_PIN_FRL 10
#define ESC_PIN_FRR 5
#define ESC_PIN_RRL 6
#define ESC_PIN_RRR 9
#define LED_PIN_RED 11
#define LED_PIN_GREEN 12
/* PWM */
#define ESC_PWM_MIN 1000
#define ESC_PWM_MAX 2000
#define IA6B_PWM_MIN 1000
#define IA6B_PWM_MAX 2000
/* ALTRO */
#define NUM_ESCs 4 // Definizione del numero di ESC

/* Parametri Software */
#define PULSEIN_READ_CYCLE 5
#define CALIBRATION_CYCLES 100
/* Costanti */
#define INVALID -255
/* Parametri LED */
#define MS_BLINK_ON 500
#define MS_BLINK_OFF 200
/* Parametri Flight Controller */
/* Input del pilota */
#define MIN_THROTTLE 0.0
#define MAX_THROTTLE 100.0
#define MIN_ROLL -MAX_ROLL
#define MAX_ROLL 10.0
#define MIN_YAW -MAX_YAW
#define MAX_YAW 20.0
#define ARM_TOLERANCE 20.0 // Tolleranza percentuale
/* Parametri PID Gyro */
#define KP_ROLL_GYRO 0.0
#define KI_ROLL_GYRO 0.0
#define KD_ROLL_GYRO 0.0
#define KP_YAW_GYRO 0.0
#define KI_YAW_GYRO 0.0
#define KD_YAW_GYRO 0.0
/* Parametri PID Angle */
#define KP_ROLL_ANGLE 0.0
#define KI_ROLL_ANGLE 0.0
#define KD_ROLL_ANGLE 0.0
#define KP_YAW_ANGLE 0.0
#define KI_YAW_ANGLE 0.0
#define KD_YAW_ANGLE 0.0

/* Debugging */
#define DEBUG_ENABLED 0
#define DEBUG_IA6B 0
#define DEBUG_PILOT_DATA 1
#define DEBUG_FLIGHT_DATA_GYRO 0
#define DEBUG_FLIGHT_DATA_ANGLE 0
#define DEBUG_ESC 0
#define DEBUG_LED 0
#define DEBUG_BNO055 0

#if DEBUG_ENABLED
#define DEBUG_PRINT(x) Serial.print(x);
#define DEBUG_PRINTLN(x) Serial.println(x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#endif // CONFIG_H