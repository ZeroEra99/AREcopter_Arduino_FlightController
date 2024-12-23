#ifndef CONFIG_H
#define CONFIG_H

// Pin di input RC
#define RC_INPUT_THROTTLE_PIN 7
#define RC_INPUT_ROLL_PIN 2
#define RC_INPUT_PITCH_PIN 4
#define RC_INPUT_HEAD_PIN 8

// Range di arm/disarm
#define ARMING_RANGE 100

// Soglie minime RC
#define MIN_RC_INPUT_ROLL 0.5
#define MIN_RC_INPUT_PITCH 0.5
#define MIN_RC_INPUT_HEAD 1
#define MIN_RC_INPUT_THROTTLE 15

// Valori massimi RC
#define MAX_RC_INPUT_ROLL 6
#define MAX_RC_INPUT_PITCH 6
#define MAX_RC_INPUT_HEAD 10
#define MAX_RC_INPUT_THROTTLE 100

// Limiti IO
#define IO_MIN 1000
#define IO_MAX 2000

#endif // CONFIG_H
