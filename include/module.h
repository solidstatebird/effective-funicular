#ifndef MODULE_H
#define MODULE_H

#include <Arduino.h>
#include <Encoder.h>
#include <FastFloatPID.h> //https://github.com/macaba/FastFloatPID

/////////////////////////////////////////////////////////////////////////
////////////////////////////   CONFIG     ///////////////////////////////
/////////////////////////////////////////////////////////////////////////

const float ANGLE_PID_SAMPLE_TIME = 0.002;
const float SPEED_PID_SAMPLE_TIME = 0.001;

const float ENCODER_TICKS_PER_REVOLUTION = 28;

const int MAX_MOTOR_OUTPUT = 255;

const float STEERING_RATIO = (41.0 / 17.0) * (41.0 / 17.0) * (97.0 / 17.0), //motor:module
            WHEEL_RATIO = STEERING_RATIO * (15.0 / 59.0) * (10.0 / 9.0); //motor:wheel

const float WHEEL_CIRCUMFERENCE_IN = PI * 2.5;

const float SPEED_KP = 20.0,
            SPEED_KI = 0.0,
            SPEED_KD = 0.0,
            ANGLE_KP = 220.0,
            ANGLE_KI = 0.0,
            ANGLE_KD = 0.0;

//module   1, 2, 3

const float MODULE_OFFSETS[] = {0.0 * DEG_TO_RAD, 120.0 * DEG_TO_RAD, -120.0 * DEG_TO_RAD};
const int_fast8_t MODULE_DIRECTIONS[] = {1, 1, -1};

const uint8_t HALLPINS[] = {A20, A21, A22};
const int MAGNET_THRESHOLDS[] = {410, 385, 410};

//////////////////////////////// PINS ////////////////////////////////

//           <module 1> <module 2> <module 3>
//             m1, m2     m1, m2     m1, m2

const uint8_t DIRPINS[3][2] = {{23, 21}, {32, 31}, {3, 5}};
const uint8_t PWMPINS[3][2] = {{22, 20}, {30, 29}, {4, 6}};

//           <module 1>      <module 2>       <module 3>
//          <m1>    <m2>    <m1>    <m2>     <m1>    <m2>
//          A,B      A,B    A,B      A,B     A,B      A,B

const uint8_t ENCODERPINS[3][2][2] = {{{18, 16}, {19, 17}}, {{26, 28}, {25, 27}}, {{8, 10}, {7, 9}}};

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////


enum ModuleID
{
    ID_MODULE1 = 0,
    ID_MODULE2 = 1,
    ID_MODULE3 = 2
};

class MotorController
{
public:
    ModuleID id;
    void setOutput(int, int);
    void zero();
    MotorController(ModuleID);

private:
    uint8_t dir1, dir2,
        pwm1, pwm2;
};

class Module
{
public:
    Module(ModuleID);
    ~Module();
    ModuleID id;

    MotorController moduleController;
    Encoder m1Encoder, m2Encoder;
    FastFloatPID speedControl, angleControl;

    void disarm();
    void arm();

    boolean home();

    void updateAngle();
    void updateSpeed();

    void setSpeed(float);
    void setAngle(float);

    int getMaxOutput();
    void updateMotorController(int);

private:
    boolean armed = false;

    const uint8_t hallPin;

    float measuredWheelPosition, measuredAngle,
        targetWheelPosition, targetAngle,
        PIDspeed, PIDangle,
        targetSpeed = 0;
};

#endif