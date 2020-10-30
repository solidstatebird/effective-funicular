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
boolean homeError = false;

void updateAngles();
void updateSpeeds();
void updateDisarmTimer();
void parsePacket();
void updateMotorOutputs();

void setup()
{
    Serial.begin(2000000);
    Serial.setTimeout(10000);

    pinMode(LED_BUILTIN, OUTPUT);
    analogWrite(LED_BUILTIN, 255);

    Radio::initialize();

    module1.arm();
    module2.arm();
    module3.arm();
    module1.setAngle(0);
    module2.setAngle(0);
    module3.setAngle(0);

    unsigned long now = millis();
    while(millis() - now < 1000)
    {
       updateSpeeds();
       updateAngles(); 
    }


}

void loop()
{
    // Serial.print(analogRead(HALLPINS[ID_MODULE1])); Serial.print("  ");
    // Serial.print(analogRead(HALLPINS[ID_MODULE2])); Serial.print("  ");
    // Serial.println(analogRead(HALLPINS[ID_MODULE3]));

    if (Serial.available() > 6)
    {
        String b = Serial.readStringUntil('\n');
        module1.setSpeed(b.substring(0, 4).toFloat());
        module1.setAngle(DEG_TO_RAD * b.substring(5, 8).toFloat());
        module2.setSpeed(b.substring(0, 4).toFloat());
        module2.setAngle(DEG_TO_RAD * b.substring(5, 8).toFloat());
        module3.setSpeed(b.substring(0, 4).toFloat());
        module3.setAngle(DEG_TO_RAD * b.substring(5, 8).toFloat());
    }


    Radio::update();
    if (Radio::packetAvailable())
    {
        parsePacket();
        disableTimer = millis();
    }
    updateSpeeds();
    updateAngles();
#warning disarm timer diabled
    //updateDisarmTimer();
}

void parsePacket()
{
    Radio::Packet packet = Radio::getLastPacket();

    if(homeError)
    {
        uint16_t responseFlags = 0;
        SETFLAG(responseFlags, Radio::RESPONSE_FLAG_HOME_ERROR);
        Radio::sendStatus(responseFlags);
    }

    if(!connected)
    {
        uint16_t responseFlags = 0;
        SETFLAG(responseFlags, Radio::RESPONSE_FLAG_BUSY);
        Radio::sendStatus(responseFlags);
        if (!module1.home() || !module2.home() || !module3.home())
            homeError = true;
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

        uint16_t responseFlags = 0;
        Radio::sendStatus(responseFlags);

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