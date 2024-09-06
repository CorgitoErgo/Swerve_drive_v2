#include "functions.h"
#include "api.h"
#include "definitions.h"

//Returns the sign of a int variable
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//Forces all motors to use brake mode
void brake(){
    leftMotors.brake();
    rightMotors.brake();
}

//Returns normalized angle for rotation sensor
int getNormalizedSensorAngle(pros::Rotation &sensor)
{
    // Convert from Centidegrees to degrees
    float angle = sensor.get_angle() / 100.0;

    if (angle < -180)
        angle += 360;
    else if (angle > 180)
        angle -= 360;

    return angle;
}

//Returns angle and taking into account of invalid values
double getAngle(int x, int y)
{
    double a = std::atan2(std::abs(y), std::abs(x));

    if (x < 0){
        if (y < 0)
            return (M_PI * -1) + a;
        else if (y > 0)
            return M_PI - a;
        else
            return M_PI;
    }
    else if (x > 0){
        if (y < 0)
            return a * -1;
        else if (y > 0)
            return a;
        else
            return 0;
    }
    else{
        if (y < 0)
            return -M_PI / 2;
        else if (y > 0)
            return M_PI / 2;
        else
            // return illegal value because a zero vector has no direction
            return 10000;
    }
}

double wrapAngle(double angle) {
    if (angle > 180.0)
        return angle -= 360;
    else if (angle < -180.0)
        return angle += 360;
    else
        return angle;
}