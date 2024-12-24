#ifndef CONFIG_H
#define CONFIG_H

// Pin di input RC
#define RC_INPUT_THROTTLE_PIN 7
#define RC_INPUT_ROLL_PIN 2
#define RC_INPUT_PITCH_PIN 4
#define RC_INPUT_HEAD_PIN 8

// Pin di output ESC
#define ESC_OUTPUT_FRL_PIN 3
#define ESC_OUTPUT_FRR_PIN 5
#define ESC_OUTPUT_BKL_PIN 6
#define ESC_OUTPUT_BKR_PIN 9

// Pin di output LED
#define RED_LED_PIN 11
#define GREEN_LED_PIN 12

// Range di arm/disarm
#define ARMING_RANGE 100

// Soglie minime RC
#define MIN_RC_INPUT_ROLL 1
#define MIN_RC_INPUT_PITCH 1
#define MIN_RC_INPUT_HEAD 0.5
#define MIN_RC_INPUT_THROTTLE 20

// Valori massimi RC
#define MAX_RC_INPUT_ROLL 10
#define MAX_RC_INPUT_PITCH 10
#define MAX_RC_INPUT_HEAD 10
#define MAX_RC_INPUT_THROTTLE 100

// Limiti IO
#define IO_MIN 1000
#define IO_MAX 2000

#endif // CONFIG_H
