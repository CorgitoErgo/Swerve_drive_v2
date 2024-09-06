#include "api.h"

template <typename T> 
int sgn(T val);
void brake();
int getNormalizedSensorAngle(pros::Rotation &sensor);
double getAngle(int x, int y);
double wrapAngle(double angle);