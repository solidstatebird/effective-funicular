#include <Arduino.h>
#include <MPU6050.h>
#include <i2c_t3.h>

#include "radio.h"
#include "module.h"
#include "moduleMath.h"
//#include "util.h"

const float GYRO_SAMPLE_TIME = 0.025;
const unsigned long SAFETY_TIMEOUT_MS = 1000;

Module module1(ID_MODULE1),
    module2(ID_MODULE2),
    module3(ID_MODULE3);
MPU6050 mpu;

boolean enabled = false;
boolean connected = false;

unsigned long disableTimer = 0;

float botAngle = 0.0; //degrees
float maxSpeed = 40.0f;



PolarCoordinates v1, v2, v3;
float preA1 = 0.0f, preA2 = 0.0f, preA3 = 0.0f;
int turns1, turns2, turns3;

CartesianCoordinates t;

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

    // Serial.begin(2000000);
    // while(1) {
    //     updateGyro();
    //     delay(20);
    //     Serial.println(botAngle);
    // }

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

    t = rotateVector(packet, botAngle);

    v1 = toWheelVelocity(packet, module1, t);
    v2 = toWheelVelocity(packet, module2, t);
    v3 = toWheelVelocity(packet, module3, t);

    normalizingSpeeds(v1.magnitude, v2.magnitude, v3.magnitude);

    v1 = angleOptimization(preA1, module1.measuredAngle, v1, turns1);
    v2 = angleOptimization(preA2, module2.measuredAngle, v2, turns2);
    v3 = angleOptimization(preA3, module3.measuredAngle, v3, turns3);

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

    if (GETFLAG(packet.flags, Radio::FLAG_SPEED))
        maxSpeed = 100.0f;
    
    else
        maxSpeed = 40.0f;


    Radio::ResponsePacket response = {};
    //placeholders
    response.flags = 0;
    Radio::sendStatus(response);

    if (enabled)
    {
        module1.setSpeed(maxSpeed * v1.magnitude);
        module2.setSpeed(maxSpeed * v2.magnitude);
        module3.setSpeed(maxSpeed * v3.magnitude);
        module1.setAngle(v1.angle);
        module2.setAngle(v2.angle);
        module3.setAngle(v3.angle);
    }
    preA1 = v1.angle;
    preA2 = v2.angle;
    preA3 = v3.angle;
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

        botAngle += norm.YAxis * GYRO_SAMPLE_TIME;
        botAngle *= DEG_TO_RAD;

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