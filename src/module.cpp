#include <Arduino.h>
#include <Encoder.h>
#include <FastFloatPID.h>

#include "module.h"

Module::Module(ModuleID id_, MotorController *mc_)
    : id(id_), moduleController(mc_)
{
    hallPin = HALLPINS[id];
    pinMode(hallPin, INPUT);
    m1Encoder = new VelocityEncoder(ENCODERPINS[id][0][0], ENCODERPINS[id][0][1]);
    m2Encoder = new VelocityEncoder(ENCODERPINS[id][1][0], ENCODERPINS[id][1][1]);
    speedControl = new FastFloatPID(&measuredSpeed, &PIDspeed, &targetSpeed,
        SPEED_KP, SPEED_KI, SPEED_KD, DIRECT);
    angleControl = new FastFloatPID(&measuredAngle, &PIDangle, &targetAngle,
        ANGLE_KP, ANGLE_KI, ANGLE_KD, DIRECT);
    speedControl->SetSampleTime(SPEED_PID_SAMPLE_TIME);
    angleControl->SetSampleTime(ANGLE_PID_SAMPLE_TIME);
    speedControl->SetOutputLimits(-MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
    angleControl->SetOutputLimits(-MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
}

void Module::updateAngle()
{
    measuredAngle = (PI * (m1Encoder->read() + m2Encoder->read()))
        / (ENCODER_TICKS_PER_REVOLUTION * STEERING_RATIO);
    //the 2 from the radian conversion and the averaging calculation cancel out
    angleControl->Compute();
}

void Module::updateSpeed()
{
    int32_t tickDifference = m1Encoder->read() - m2Encoder->read();

    measuredSpeed = (WHEEL_CIRCUMFERENCE_IN * (tickDifference - lastTickDifference))
        / (ENCODER_TICKS_PER_REVOLUTION * WHEEL_RATIO * SPEED_PID_SAMPLE_TIME);

    lastTickDifference = tickDifference;
    speedControl->Compute();
}

boolean Module::home()
{
    if (armed)
        disarm();
    targetAngle = 0;

    unsigned long startTime = millis();

    while (1)
    {
        moduleController->setOutput(30, 30);

        if (analogRead(hallPin) < MAGNET_THRESHOLDS[id])
        {
            moduleController->setOutput(0, 0);
            m1Encoder->write(0);
            m2Encoder->write(0);
            return true;
        }
        if (millis() - startTime > 3000)
        {
            moduleController->setOutput(0, 0);
            m1Encoder->write(0);
            m2Encoder->write(0);
            return false;
        }
    }
}

void Module::updateMotorController(int max)
{
    max = abs(max);

    float m1 = -PIDspeed;
    float m2 = PIDspeed;
    m1 += PIDangle;
    m2 += PIDangle;

    moduleController->setOutput((int)(MAX_MOTOR_OUTPUT * (m1 / max)),
        (int)(MAX_MOTOR_OUTPUT * (m2 / max)));
}

int Module::getMaxOutput()
{
    int m1 = -PIDspeed;
    int m2 = PIDspeed;
    m1 += PIDangle;
    m2 += PIDangle;

    if (abs(m1) > abs(m2))
        return m1;
    else
        return m2;
}

void Module::disarm()
{
    angleControl->SetMode(MANUAL);
    speedControl->SetMode(MANUAL);
    targetSpeed = 0;
    PIDangle = 0;
    PIDspeed = 0;
    moduleController->zero();
    armed = false;
}

void Module::arm()
{
    angleControl->SetMode(AUTOMATIC);
    speedControl->SetMode(AUTOMATIC);
    armed = true;
}

void Module::setAngle(float setpoint)
{
    targetAngle = setpoint;
}

void Module::setSpeed(float setpoint)
{
    //prevent the speed PID from sending an output in the wrong direction
    if (setpoint >= 0)
    {
        speedControl->SetOutputLimits(0, MAX_MOTOR_OUTPUT);
    }
    else
    {
        speedControl->SetOutputLimits(-MAX_MOTOR_OUTPUT, 0);
    }
    //clear any I error accumulation when desired speed is zero
    if (setpoint == 0)
    {
        speedControl->SetMode(MANUAL);
        speedControl->SetMode(AUTOMATIC);
    }

    targetSpeed = setpoint;
}

MotorController::MotorController(ModuleID id_)
    : id(id_)
{
    dir1 = DIRPINS[id][0];
    dir2 = DIRPINS[id][1];
    pwm1 = PWMPINS[id][0];
    pwm2 = PWMPINS[id][1];
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(pwm1, OUTPUT);
    pinMode(pwm2, OUTPUT);
    analogWriteFrequency(pwm1, 15000);
    analogWriteFrequency(pwm2, 15000);
}

void MotorController::setOutput(int m1, int m2)
{
    if (m1 >= 0)
    {
        digitalWrite(dir1, LOW);
        analogWrite(pwm1, m1);
    }
    else
    {
        digitalWrite(dir1, HIGH);
        analogWrite(pwm1, abs(m1));
    }
    if (m2 >= 0)
    {
        digitalWrite(dir2, LOW);
        analogWrite(pwm2, m2);
    }
    else
    {
        digitalWrite(dir2, HIGH);
        analogWrite(pwm2, abs(m2));
    }
}

void MotorController::zero()
{
    analogWrite(pwm1, 0);
    analogWrite(pwm2, 0);
}