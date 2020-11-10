#include <Arduino.h>

#include "radio.h"
#include "module.h"

//#include "util.h"

const unsigned long SAFETY_TIMEOUT_MS = 1000;
unsigned long disableTimer = 0;

Module module1(ID_MODULE1),
    module2(ID_MODULE2),
    module3(ID_MODULE3);

boolean enabled = false;
boolean connected = false;

void updateAngles();
void updateSpeeds();
void updateDisarmTimer();
void parsePacket();
void updateMotorOutputs();

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Radio::initialize();
}

void loop()
{
    Radio::update();
    if (Radio::packetAvailable())
    {
        parsePacket();
        disableTimer = millis();
    }
    updateSpeeds();
    updateAngles();
    //#warning disarm timer diabled
    updateDisarmTimer();
}

void parsePacket()
{
    Radio::Packet packet = Radio::getLastPacket();

    if(!connected)
    {
         Radio::ResponsePacket response = {};
        SETFLAG(response.flags, Radio::RESPONSE_FLAG_BUSY);
        Radio::sendStatus(response);
        if (!module1.home() || !module2.home() || !module3.home())
            connected = false;
        else
            connected = true;
    }

    if (GETFLAG(packet.flags, Radio::FLAG_ENABLE))
    {
        if (!enabled)
        {
            module1.arm();
            module2.arm();
            module3.arm();
            enabled = true;
        }
    }
    else
    {
        module1.disarm();
        module2.disarm();
        module3.disarm();
        enabled = false;
    }

    Radio::ResponsePacket response = {};
    //placeholders
    response.angle = 0;
    response.flags = 0;
    Radio::sendStatus(response);

    if (enabled)
    {
        module1.setSpeed(packet.s1);
        module2.setSpeed(packet.s2);
        module3.setSpeed(packet.s3);
        module1.setAngle(packet.a1);
        module2.setAngle(packet.a2);
        module3.setAngle(packet.a3);
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

void updateMotorOutputs()
{
    int x = abs(module1.getMaxOutput());
    int y = abs(module2.getMaxOutput());
    int z = abs(module3.getMaxOutput());
    if (x < y)
        x = y;
    if (x < z)
        x = z;
    if (x <= MAX_MOTOR_OUTPUT)
        x = MAX_MOTOR_OUTPUT;
    module1.updateMotorController(x);
    module2.updateMotorController(x);
    module3.updateMotorController(x);
}

void updateDisarmTimer()
{
    if (millis() - disableTimer > SAFETY_TIMEOUT_MS)
    {
        module1.disarm();
        module2.disarm();
        module3.disarm();
        enabled = false;
        connected = false;
    }
}