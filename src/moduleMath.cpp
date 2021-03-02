#include "radio_defs.h"
#include "module.h"
#include "moduleMath.h"

CartesianCoordinates PolarCoordinates::toCartesian() {
    CartesianCoordinates returnCoord(magnitude * cos(angle), magnitude * sin(angle));
    return returnCoord;
}

float ConstrainedAngle(float angle)
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

CartesianCoordinates rotateVector(CartesianCoordinates old, float angle){
    CartesianCoordinates t;
    t.x = old.x * cosf(-angle) - old.y * sinf(-angle);
    t.y = old.x * sinf(-angle) + old.y * cosf(-angle);
    return t;
}
PolarCoordinates toWheelVelocity(float wp, Module m, CartesianCoordinates t){
    CartesianCoordinates w;
    CartesianCoordinates temp;
    PolarCoordinates v;

    if (m.id == 0){
        w.x = wp;
        w.y = 0.0f;
    }
    else if (m.id == 1){
        w.x = -0.5*wp;
        w.y = -0.866*wp;
    }
    else{
        w.x = -0.5*wp;
        w.y = 0.866*wp;
    }

    temp.x = t.x + w.x;
    temp.y = t.y + w.y;

    v = temp.toPolar();
    return v;
}

float angleDiff(float a,float b){
    float dif = fmodf(b - a + F_PI,2*F_PI);
    if (dif < 0)
        dif += 2 * F_PI;
    return dif - F_PI;
}
float unwrap(float previousAngle,float newAngle){
    return previousAngle - angleDiff(newAngle,ConstrainedAngle(previousAngle));
}

void normalizingSpeeds(float& s1, float& s2 , float& s3)
{
    float n = s1;
    if (n < s2)
        n = s2;
    if (n < s3)
        n = s3;
    if (n < 1.0f)
        n = 1.0f;
    s1 /= n;
    s2 /= n;
    s3 /= n;
}
PolarCoordinates angleOptimization(float currentA, PolarCoordinates v){
    
    if (abs(v.magnitude) < 0.05){
        v.angle = currentA;
        return v;
    }


    float angleDifference = abs(v.angle - currentA);

    float cosError = cosf(angleDifference);    

    v.magnitude *= powf(cosError, 5);

    if (cosError < 0){
        v.angle = ConstrainedAngle(v.angle + F_PI);    
    }

    
    v.angle = unwrap(currentA, v.angle);

    return v;
}
PolarCoordinates CartesianCoordinates::toPolar() {
    float angle;
    if (x == 0.0f && y == 0.0f)
        angle = 0.0f;
    else
        angle = ConstrainedAngle(atan2(-x, y));
    PolarCoordinates returnCoord(sqrt(x * x + y * y), angle);
    return returnCoord;
}
