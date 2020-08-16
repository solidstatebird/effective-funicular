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
            WHEEL_RATIO = STEERING_RATIO * (15.0 / 59.0); //motor:wheel

const float WHEEL_CIRCUMFERENCE_IN = PI * 2.5;

const float SPEED_KP = 20,
            SPEED_KI = 0,
            SPEED_KD = 0.0,
            ANGLE_KP = 220.0,
            ANGLE_KI = 0.0,
            ANGLE_KD = 0.0;

//module   1, 2, 3

const int MAGNET_THRESHOLDS[] = {25, 50, 200};

//////////////////////////////// PINS ////////////////////////////////

//           <module 1> <module 2>
//             m1, m2     m1, m2

const uint8_t DIRPINS[3][2] = {{21, 20}, {31, 32}, {5, 6}};
const uint8_t PWMPINS[3][2] = {{23, 22}, {29, 30}, {3, 4}};

const uint8_t HALLPINS[] = {A15, A14, A16};

//           <module 1>      <module 2>       <module 3>
//          <m1>    <m2>    <m1>    <m2>     <m1>    <m2>
//          A,B      A,B    A,B     A,B      A,B     A,B

const uint8_t ENCODERPINS[3][2][2] = {{{19, 18}, {17, 16}}, {{25, 26}, {27, 28}}, {{7, 8}, {9, 10}}};

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

    MotorController *moduleController;
    Encoder *m1Encoder, *m2Encoder;
    FastFloatPID *speedControl, *angleControl;

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

    uint8_t hallPin;

    float measuredWheelPosition, measuredAngle,
        targetWheelPosition, targetAngle,
        PIDspeed, PIDangle,
        targetSpeed = 0;
};

#endif