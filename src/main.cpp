#include <Arduino.h>
#include <MPU6050.h>

#include "radio.h"
#include "module.h"

//#include "util.h"

const float GYRO_SAMPLE_TIME = 0.01;
const unsigned long SAFETY_TIMEOUT_MS = 1000;

Module module1(ID_MODULE1),
    module2(ID_MODULE2),
    module3(ID_MODULE3);
MPU6050 mpu;

boolean enabled = false;
boolean connected = false;

unsigned long disableTimer = 0;

float botAngle = 0.0; //degrees

void updateAngles();
void updateGyro();
void updateSpeeds();
void blinky();
void updateDisarmTimer();
void parsePacket();
void updateMotorOutputs();

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    mpu.begin(MPU6050_SCALE_500DPS, MPU6050_RANGE_2G);
    delay(500);
    mpu.calibrateGyro();
    mpu.setThreshold(0);

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
    updateGyro();
    blinky();
    updateSpeeds();
    updateAngles();
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
        {
            connected = true;
            botAngle = 0.0;
        }
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
    response.angle = botAngle * DEG_TO_RAD;
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
    static unsigned long lastCalc = micros();
    if (micros() - lastCalc > 1e6 * ANGLE_PID_SAMPLE_TIME)
    { //this syntax still works through a timer overflow
        module1.updateAngle();
        module2.updateAngle();
        module3.updateAngle();

        updateMotorOutputs();

        lastCalc = micros();
    }
}

void updateGyro()
{
    static unsigned long lastCalc = micros();
    if (micros() - lastCalc > 1e6 * GYRO_SAMPLE_TIME)
    { //this syntax still works through a timer overflow
        Vector norm = mpu.readNormalizeGyro();

        botAngle = botAngle + norm.YAxis * GYRO_SAMPLE_TIME;

        lastCalc = micros();
    }
}

void updateSpeeds()
{
    static unsigned long lastCalc = micros();
    if (micros() - lastCalc > 1e6 * SPEED_PID_SAMPLE_TIME)
    { //this syntax still works through a timer overflow
        module1.updateSpeed();
        module2.updateSpeed();
        module3.updateSpeed();

        updateMotorOutputs();

        lastCalc = micros();
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

void blinky()
{
    static unsigned long lastBlink = millis();
    static boolean state = false;
    if(!enabled)
        digitalWrite(LED_BUILTIN, HIGH);
    else
    {
        if(millis() - lastBlink > 250)
        {
            if(state)
                digitalWrite(LED_BUILTIN, HIGH);
            else
                digitalWrite(LED_BUILTIN, LOW);
            state = !state;
            lastBlink = millis();
        }
    }
    
}