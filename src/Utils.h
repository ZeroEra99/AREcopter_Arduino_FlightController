#ifndef UTILS_H
#define UTILS_H

template <typename T, typename U, typename V>
bool isInRange(T value, U min, V max)
{
    // Determiniamo il minimo e il massimo indipendentemente dall'ordine
    T lower = (min <= max) ? min : max;
    T upper = (min <= max) ? max : min;

    // Controlliamo se il valore è nel range
    return value >= lower && value <= upper;
}

int digital_to_pwm(double value, double min_digital, double max_digital, int min_analog, int max_analog);
double pwm_to_digital(int pwm_value, int min_analog, int max_analog, double min_digital, double max_digital);

#endif // UTILS_H