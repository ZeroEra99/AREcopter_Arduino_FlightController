#include "Utils.h"

int digital_to_pwm(double value, double min_digital, double max_digital, int min_analog, int max_analog)
{
    // Formula di mappatura lineare per ottenere il valore PWM
    int pwm_value = (int)((value - min_digital) * (max_analog - min_analog) / (max_digital - min_digital) + min_analog);

    // Verifica che il valore sia compreso nell'intervallo analogico
    if (pwm_value < min_analog)
    {
        pwm_value = min_analog;
    }
    else if (pwm_value > max_analog)
    {
        pwm_value = max_analog;
    }

    return pwm_value;
}

double pwm_to_digital(int pwm_value, int min_analog, int max_analog, double min_digital, double max_digital)
{
    // Mappatura lineare del valore PWM nel range digitale
    double digital_value = (double)(pwm_value - min_analog) / (max_analog - min_analog) * (max_digital - min_digital) + min_digital;
    // Verifica che il valore sia compreso nell'intervallo digitale
    if (digital_value < min_digital)
    {
        digital_value = min_digital;
    }
    else if (digital_value > max_digital)
    {
        digital_value = max_digital;
    }
    return digital_value;
}