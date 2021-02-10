#include "radio_defs.h"
#include "module.h"
#include "moduleMath.h"


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

CartesianCoordinates rotateVector(Radio::Packet p, float angle){
    CartesianCoordinates t;
    t.x = p.tx * cosf(angle) - p.ty * sinf(angle);
    t.y = p.tx * sinf(angle) + p.ty * cosf(angle);
    return t;
}
PolarCoordinates toWheelVelocity(Radio::Packet p, Module m, CartesianCoordinates t){
    CartesianCoordinates w;
    CartesianCoordinates temp;
    PolarCoordinates v;

    if (m.id == 0){
        w.x = p.w;
        w.y = 0.0f;
    }
    else if (m.id == 1){
        w.x = -0.5*p.w;
        w.y = -0.866*p.w;
    }
    else{
        w.x = -0.5*p.w;
        w.y = 0.866*p.w;
    }

    temp.x = t.x + w.x;
    temp.y = t.y + w.y;

    v = temp.toPolar();
    return v;
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
PolarCoordinates angleOptimization(float preA, float currentA, PolarCoordinates v, int& turns){
    if ((preA-turns*2*F_PI) - F_PI > v.angle)
        turns++;
    else if ((preA-turns*2*F_PI) + F_PI < v.angle)
        turns--;
    v.angle += 2 * F_PI * turns;
    if (abs(v.magnitude) < 0.01)
        v.angle = preA;
    
    float cosError = cosf(abs(v.angle - preA));    
    v.magnitude *= cosError;

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
