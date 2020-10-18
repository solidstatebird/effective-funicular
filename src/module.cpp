#include <Arduino.h>
#include <Encoder.h>
#include <FastFloatPID.h>

#include "module.h"

Module::Module(ModuleID id_)
    : id(id_),
    moduleController(id),
    m1Encoder(ENCODERPINS[id][0][0], ENCODERPINS[id][0][1]),
    m2Encoder(ENCODERPINS[id][1][0], ENCODERPINS[id][1][1]),
    speedControl(&measuredWheelPosition, &PIDspeed, &targetWheelPosition,
                                    SPEED_KP, SPEED_KI, SPEED_KD, DIRECT),
    angleControl(&measuredAngle, &PIDangle, &targetAngle,
                                    ANGLE_KP, ANGLE_KI, ANGLE_KD, DIRECT),
    hallPin(HALLPINS[id])
{
    pinMode(hallPin, INPUT);
    speedControl.SetSampleTime(SPEED_PID_SAMPLE_TIME);
    angleControl.SetSampleTime(ANGLE_PID_SAMPLE_TIME);
    speedControl.SetOutputLimits(-MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
    angleControl.SetOutputLimits(-MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
}

Module::~Module() {}

void Module::updateAngle()
{
    if(!armed) return;
    measuredAngle = (PI * (m1Encoder.read() + m2Encoder.read())) / (ENCODER_TICKS_PER_REVOLUTION * STEERING_RATIO);
    //the 2 from the radian conversion and the averaging calculation cancel out
    angleControl.Compute();
}

void Module::updateSpeed()
{
    if(!armed) return;
    int32_t ticks1 = m1Encoder.read();
    int32_t ticks2 = m2Encoder.read();

    measuredWheelPosition = ticks1 - ticks2;
    
    targetWheelPosition += targetSpeed * SPEED_PID_SAMPLE_TIME ;

    speedControl.Compute();
}

boolean Module::home()
{
    if (armed)
        disarm();
    targetAngle = 0;

    unsigned long startTime = millis();

    while (1)
    {
        moduleController.setOutput(40, 40);

        if (analogRead(hallPin) > MAGNET_THRESHOLDS[id])
        {
            moduleController.setOutput(0, 0);
            m1Encoder.write(0);
            m2Encoder.write(0);
            return true;
        }
        if (millis() - startTime > 5000)
        {
            moduleController.setOutput(0, 0);
            m1Encoder.write(0);
            m2Encoder.write(0);
            return false;
        }
    }
}

void Module::updateMotorController(int max)
{
    if(!armed) {
        moduleController.zero();
        return;
    }
    
    max = abs(max);

    float m1 = -PIDspeed;
    float m2 = PIDspeed;
    m1 += PIDangle;
    m2 += PIDangle;

    moduleController.setOutput((int)(MAX_MOTOR_OUTPUT * (m1 / max)),
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
    angleControl.SetMode(MANUAL);
    speedControl.SetMode(MANUAL);
    targetSpeed = 0;
    PIDangle = 0;
    PIDspeed = 0;
    moduleController.zero();
    armed = false;
}

void Module::arm()
{

    angleControl.SetMode(AUTOMATIC);
    speedControl.SetMode(AUTOMATIC);
    armed = true;
}

void Module::setAngle(float setpoint)
{
    targetAngle = setpoint;
}

void Module::setSpeed(float setpoint)
{
    targetSpeed = setpoint * ((ENCODER_TICKS_PER_REVOLUTION * WHEEL_RATIO) / WHEEL_CIRCUMFERENCE_IN); //in/sec to ticks/sec
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