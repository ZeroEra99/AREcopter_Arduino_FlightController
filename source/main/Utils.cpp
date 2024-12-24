#include "Utils.h"

// Funzione per calcolare la differenza angolare normalizzata tra due angoli
float normalizeAngle(float angle) {
  while (angle > 180.0f) {
    angle -= 360.0f;
  }
  while (angle < -180.0f) {
    angle += 360.0f;
  }
  return angle;
}