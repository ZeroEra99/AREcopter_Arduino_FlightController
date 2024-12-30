#ifndef CONFIG_H
#define CONFIG_H

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

// Range di arm/disarm
#define ARMING_RANGE 120

// Soglie minime RC
#define MIN_RC_INPUT_ROLL 0
#define MIN_RC_INPUT_PITCH 0
#define MIN_RC_INPUT_HEAD 0
#define MIN_RC_INPUT_THROTTLE 8

// Valori massimi RC
#define MAX_RC_INPUT_ROLL 15
#define MAX_RC_INPUT_PITCH 15
#define MAX_RC_INPUT_2AXIS_COMBINED 10
#define MAX_RC_INPUT_HEAD 10
#define MAX_RC_INPUT_THROTTLE 100

// Valori PID
#define PID_VALUE_ROLL_P 0.16 //.16
#define PID_VALUE_ROLL_I 0.008 //.008
#define PID_VALUE_ROLL_D 0.06 //.06
#define PID_VALUE_PITCH_P 0.12//.22
#define PID_VALUE_PITCH_I 0.005//.01
#define PID_VALUE_PITCH_D 0.06//.06
#define PID_VALUE_HEAD_P 1//.5
#define PID_VALUE_HEAD_I 0.0//.01
#define PID_VALUE_HEAD_D 0.0//.008
// Valori massimi PID
#define MAX_DIFF_INPUT_HEAD 40

// Limiti IO
#define IO_MIN 1000
#define IO_MAX 2000

#endif // CONFIG_H
