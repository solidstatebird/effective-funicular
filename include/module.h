#ifndef MODULE_H
#define MODULE_H

#include <Arduino.h>
#include <Encoder.h>
#include <FastFloatPID.h>

/////////////////////////////////////////////////////////////////////////
////////////////////////////   CONFIG     ///////////////////////////////
/////////////////////////////////////////////////////////////////////////
const float ANGLE_PID_SAMPLE_TIME = 0.002;
const float SPEED_PID_SAMPLE_TIME = 0.02;

const float ENCODER_TICKS_PER_REVOLUTION = 28;

const int MAX_MOTOR_OUTPUT = 255;

const float STEERING_RATIO = (49.0 / 25.0) * (49.0 / 25.0) * (97.0 / 25.0),
            WHEEL_RATIO = STEERING_RATIO * (15.0 / 59.0);

const float WHEEL_CIRCUMFERENCE_IN = PI * 2.5;

const float SPEED_KP = 1.2,
            SPEED_KI = 2.0,
            SPEED_KD = 0.0,
            ANGLE_KP = 110.0,
            ANGLE_KI = 0.0,
            ANGLE_KD = 0.0;

const int MAGNET_THRESHOLD = 80;

//PINS

const uint8_t MOD1_M1_DIRPIN = 21, MOD1_M1_PWMPIN = 23,
              MOD1_M2_DIRPIN = 20, MOD1_M2_PWMPIN = 22,
              MOD2_M1_DIRPIN = 31, MOD2_M1_PWMPIN = 29,
              MOD2_M2_DIRPIN = 32, MOD2_M2_PWMPIN = 30,
              MOD3_M1_DIRPIN = 5, MOD3_M1_PWMPIN = 3,
              MOD3_M2_DIRPIN = 6, MOD3_M2_PWMPIN = 4;

const uint8_t MOD1_HALLPIN = A16,
              MOD2_HALLPIN = A15,
              MOD3_HALLPIN = A14;

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

enum moduleID
{
    MODULE_1 = 1,
    MODULE_2 = 2,
    MODULE_3 = 3
};

class MotorController
{
public:
    MotorController(uint8_t, uint8_t, uint8_t, uint8_t);
    void setOutput(int, int);
    void zero();

private:
    uint8_t dir1, dir2,
        pwm1, pwm2;
};

class Module
{
public:
    moduleID id;

    FastFloatPID *speedControl, *angleControl;
    Encoder *m1Encoder, *m2Encoder;
    MotorController *moduleController;

    void disarm();
    void arm();

    boolean home();

    void updateAngle();
    void updateSpeed();

    void setSpeed(float);
    void setAngle(float);

    int getMaxOutput();
    void updateMotorController(int);

    Module(moduleID, Encoder *, Encoder *, MotorController *, uint8_t);

private:
    boolean armed = false;

    uint8_t hallPin;

    float measuredSpeed, measuredAngle,
        targetSpeed, targetAngle,
        PIDspeed, PIDangle = 0;

    int32_t lastTickDifference = 0;
};

#endif