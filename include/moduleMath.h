#include "botConstants.h"

class CartesianCoordinates;

class PolarCoordinates {
public:
    float getConstrainedAngle() const
    {
        float tempAngle = angle;
        if (tempAngle < 0)
        {
            tempAngle = fmod(tempAngle, 2 * F_PI);
            tempAngle += 2 * F_PI;
        }
        if (tempAngle > 2 * F_PI)
        {
            tempAngle = fmod(tempAngle, 2 * F_PI);
        }
        return tempAngle;
    }

    PolarCoordinates operator-()
    {
        return PolarCoordinates(-magnitude, angle);
    }

    CartesianCoordinates toCartesian();

    PolarCoordinates(): magnitude(0), angle(0) {}
    PolarCoordinates(float magnitude_, float angle_): magnitude(magnitude_), angle(angle_) {}

public:
    float magnitude;
    float angle;
};

class CartesianCoordinates {
public:
    PolarCoordinates toPolar();
    CartesianCoordinates(): x(0), y(0) {}
    CartesianCoordinates(float x_, float y_) : x(x_), y(y_) {}
public:
    float x;
    float y;
};

float ConstrainedAngle(float angle);
CartesianCoordinates rotateVector(Radio::Packet p, float botAngle);
PolarCoordinates toWheelVelocity(Radio::Packet, Module, CartesianCoordinates);
void normalizingSpeeds(float& s1, float& s2 , float& s3);
PolarCoordinates angleOptimization(float, float, PolarCoordinates, int&);
