#ifndef CONFIG_H
#define CONFIG_H

/* Parametri HARDWARE */
// Pin di input RC
#define RC_INPUT_THROTTLE_PIN 3
#define RC_INPUT_ROLL_PIN 2
#define RC_INPUT_PITCH_PIN 4
#define RC_INPUT_HEAD_PIN 8
// Pin di output ESC
#define ESC_OUTPUT_FRL_PIN 10
#define ESC_OUTPUT_FRR_PIN 5
#define ESC_OUTPUT_BKL_PIN 6
#define ESC_OUTPUT_BKR_PIN 9
// Pin di output LED
#define RED_LED_PIN 11
#define GREEN_LED_PIN 12

/* Parametri I/O Analogici */
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_ARMING_TOLERANCE 150

/* Parametri Input pilota */
#define IDLE_THROTTLE 15        // Potenza % assegnata ai motori con drone armato
#define MAX_THROTTLE 70         // Massima potenza % permessa
#define MAX_ANGLE_RC_INPUT 3.5  // Angolo massimo di inclinazione (Pitch e Roll)
#define MAX_HEADING_OFFSET 10   // Offset massimo di rotazione (Heading)
#define ARMING_RANGE 150        // Range di arm/disarm

/* Valori PID Pitch e Roll */
// PID Angolo 0.3 0.005 0.004 Sembra ok (Test più recente)
// Indiano: 2 0.5 0.007
#define PID_VALUE_P_ANGLE 0.3
#define PID_VALUE_I_ANGLE 0.005
#define PID_VALUE_D_ANGLE 0.004
// PID Velocità rotazione - 0.8 0.1 0.05 Sembra ok (Test più recente)
// Indiano: 0.625 2.1 0.0088
#define PID_VALUE_P_RATE 0.8
#define PID_VALUE_I_RATE 0.1
#define PID_VALUE_D_RATE 0.05
/* Valori PID Heading */
// PID Angolo
#define PID_VALUE_HEAD_P_ANGLE 1
#define PID_VALUE_HEAD_I_ANGLE 0
#define PID_VALUE_HEAD_D_ANGLE 0
// PID Velocità rotazione
#define PID_VALUE_HEAD_P_RATE 1
#define PID_VALUE_HEAD_I_RATE 0
#define PID_VALUE_HEAD_D_RATE 0

/* Parametri PID */
#define PID_PROPORZIONALITA_THROTTLE 100
// Limite Integrale PID Pitch e Roll
#define MAX_PROPORTIONAL_ANGLE 100
#define MAX_INTEGRAL_ANGLE 100
#define MAX_DERIVATIVE_ANGLE 100
#define MAX_PROPORTIONAL_RATE 100
#define MAX_INTEGRAL_RATE 100
#define MAX_DERIVATIVE_RATE 100
// Limite Integrale PID Head
#define MAX_PROPORTIONAL_HEAD_ANGLE 100
#define MAX_INTEGRAL_HEAD_ANGLE 100
#define MAX_DERIVATIVE_HEAD_ANGLE 100
#define MAX_PROPORTIONAL_HEAD_RATE 100
#define MAX_INTEGRAL_HEAD_RATE 100
#define MAX_DERIVATIVE_HEAD_RATE 100

#endif  // CONFIG_H
