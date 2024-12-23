#ifndef UTILS_H
#define UTILS_H

// Funzione per mappare un valore float da un intervallo ad un altro
inline float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif // UTILS_H
