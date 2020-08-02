#include <Arduino.h>

#include <Encoder.h>
#include <FastFloatPID.h> //https://github.com/macaba/FastFloatPID

#include "radio.h"
#include "module.h"

//#include "util.h"

const unsigned long SAFETY_TIMEOUT_MS = 1000;

MotorController module1Conroller(ID_MODULE1),
    module2Conroller(ID_MODULE2),
    module3Conroller(ID_MODULE3);

Module module1(ID_MODULE1, &module1Conroller),
    module2(ID_MODULE2, &module2Conroller),
    module3(ID_MODULE3, &module3Conroller);

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

    //module1.arm();
    // module2.arm();
    // module3.arm();
}

void loop()
{
    // Serial.print(analogRead(HALLPINS[ID_MODULE1])); Serial.print("  ");
    // Serial.print(analogRead(HALLPINS[ID_MODULE2])); Serial.print("  ");
    // Serial.println(analogRead(HALLPINS[ID_MODULE3]));

    // //radioUpdate();
    // // if (radioPacketAvailable())
    // //     parsePacket();
    // #warning disarm timer diabled
    // //updateDisarmTimer();
    // updateSpeeds();
    // updateAngles();
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