#include <Arduino.h>

#include <Encoder.h>
#include <FastFloatPID.h> //https://github.com/macaba/FastFloatPID

#include "radio.h"
#include "module.h"

//#include "util.h"

const unsigned long SAFETY_TIMEOUT_MS = 1000;

Encoder MOD1_M1_ENCODER(9, 10), MOD1_M2_ENCODER(11, 12),
    MOD2_M1_ENCODER(13, 14), MOD2_M2_ENCODER(15, 16),
    MOD3_M1_ENCODER(17, 18), MOD3_M2_ENCODER(19, 20);

MotorController module1Conroller(MOD1_M1_DIRPIN, MOD1_M2_DIRPIN, MOD1_M1_PWMPIN, MOD1_M2_PWMPIN),
    module2Conroller(MOD2_M1_DIRPIN, MOD2_M2_DIRPIN, MOD2_M1_PWMPIN, MOD2_M2_PWMPIN),
    module3Conroller(MOD3_M1_DIRPIN, MOD3_M2_DIRPIN, MOD3_M1_PWMPIN, MOD3_M2_PWMPIN);

Module module1(MODULE_1, &MOD1_M1_ENCODER, &MOD1_M2_ENCODER, &module1Conroller, MOD1_HALLPIN),
    module2(MODULE_2, &MOD2_M1_ENCODER, &MOD2_M2_ENCODER, &module2Conroller, MOD2_HALLPIN),
    module3(MODULE_3, &MOD3_M1_ENCODER, &MOD3_M2_ENCODER, &module3Conroller, MOD3_HALLPIN);

boolean enabled = false;

void updateDisarmTimer();
void updateAngles();
void updateSpeeds();
void parsePacket();
void updateMotorOutputs();

void setup()
{
    Serial.begin(2000000);
    Serial.setTimeout(10000);

    radioInitialize();

    // module1.home();
    // module2.home();
    // module3.home();

    module1.arm();
    // module2.arm();
    // module3.arm();
}

void loop()
{
    // module1.setSpeed(30.0);
    // static unsigned long lastANGLECHANGE = 0;
    // if(millis() - lastANGLECHANGE > 1000) {
    //     // mod1_targetangle = 3.14 * sin(millis() * 0.00328);
    //     // mod1_targetspeed = 60 * sin(HALF_PI/2 + millis() * 0.00328);
    //     static float angle = 0;
    //     module1.setAngle(angle + (90* DEG_TO_RAD));
    //     lastANGLECHANGE = millis();
    // }

    if (Serial.available() > 6)
    {
        String b = Serial.readStringUntil('\n');
        module1.setSpeed(b.substring(0, 4).toFloat());
        module1.setAngle(DEG_TO_RAD * b.substring(5, 8).toFloat());
    }

    //radioUpdate();
    #warning disarm timer diabled
    //updateDisarmTimer();
    updateSpeeds();
    updateAngles();
    if (radioPacketAvailable())
        parsePacket();

}

void parsePacket()
{
    Packet packet = radioGetPacket();
    if (GETFLAG(packet.flags, FLAG_ENABLE))
    {
        if (!enabled)
        {
            module1.arm();
            module2.arm();
            module3.arm();
            enabled = true;
        }
    }

    if (GETFLAG(packet.flags, FLAG_ACK))
    {
        radioSendStatus();
    }

    if (enabled)
    {
        module1.setSpeed(packet.s1);
        module2.setSpeed(packet.s2);
        module3.setSpeed(packet.s3);
        module1.setAngle(packet.s1);
        module2.setAngle(packet.s2);
        module3.setAngle(packet.s3);
    }
}

void updateAngles()
{
    static unsigned long lastAngleCalc = micros();
    if (micros() - lastAngleCalc > 1e6 * ANGLE_PID_SAMPLE_TIME)
    { //this syntax still works through a timer overflow
        module1.updateAngle();
        module2.updateAngle();
        module3.updateAngle();

        updateMotorOutputs();

        lastAngleCalc = micros();
    }
}

void updateSpeeds()
{
    static unsigned long lastSpeedCalc = micros();
    if (micros() - lastSpeedCalc > 1e6 * SPEED_PID_SAMPLE_TIME)
    { //this syntax still works through a timer overflow
        module1.updateSpeed();
        module2.updateSpeed();
        module3.updateSpeed();

        updateMotorOutputs();

        lastSpeedCalc = micros();
    }
}

void updateMotorOutputs() {
    int x = abs(module1.getMaxOutput());
    int y = abs(module2.getMaxOutput());
    int z = abs(module3.getMaxOutput());
    if (x < y)
        x = y;
    if (x < z)
        x = z;
    if (x <= MAX_MOTOR_OUTPUT)
        x = 255;
    module1.updateMotorController(x);
    module2.updateMotorController(x);
    module3.updateMotorController(x);
}

void updateDisarmTimer()
{
    static unsigned long disableTimer = millis();
    if (millis() - disableTimer > SAFETY_TIMEOUT_MS)
    {
        module1.disarm();
        module2.disarm();
        module3.disarm();
        enabled = false;
    }
}