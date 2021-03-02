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

float botAngle = 0.0; //radians
float maxSpeed = 40.0f;

unsigned long measuredTime;

Radio::Packet packet;

PolarCoordinates v1, v2, v3;
float modOffset1 = 0.0f, modOffset2 = 0.0f, modOffset3 = 0.0f;

float testPrint = 1.0f;

CartesianCoordinates t;

CartesianCoordinates currentPosition;


bool skipPacket = false;

void updateAngles();
void updateGyro();
void updateSpeeds();
void blinky();
void updateDisarmTimer();
void parsePacket();
void gamepadMode(Radio::Packet);
void positionMode(Radio::Packet);
void updateMotorOutputs();


void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    mpu.begin(MPU6050_SCALE_500DPS, MPU6050_RANGE_2G);
    delay(500);
    mpu.calibrateGyro();
    mpu.setThreshold(0);

    // int preHall = 0.0f;
    // int big = 0.0f;
    // Serial.begin(2000000);
    // while(1) {
    //      int i = analogRead(module3.hallPin);
    //      if (i > preHall)
    //         big = i;

    //     Serial.println(big);
    //     delay(20);
    //     preHall = big;

    // }

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
    //if (!skipPacket){
    packet = Radio::getLastPacket();
    //}
    if (!GETFLAG(packet.flags, Radio::FLAG_MODE)){ // Gamepad Mode is 0, Position Mode is 1
        gamepadMode(packet);
    }

    else {
        positionMode(packet);
    }



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
    response.floatTest = currentPosition.x;
    Radio::sendStatus(response);

    if (enabled)
    {
        module1.setSpeed(maxSpeed*v1.magnitude);
        module2.setSpeed(maxSpeed*v2.magnitude);
        module3.setSpeed(maxSpeed*v3.magnitude);
        module1.setAngle(v1.angle);
        module2.setAngle(v2.angle);
        module3.setAngle(v3.angle);
    }
}

void gamepadMode(Radio::Packet packet){
    t.x = packet.tx;
    t.y = packet.ty;
    t = rotateVector(t, botAngle * DEG_TO_RAD);

    packet.w *= 30/maxSpeed;

    v1 = toWheelVelocity(packet.w, module1, t);
    v2 = toWheelVelocity(packet.w, module2, t);
    v3 = toWheelVelocity(packet.w, module3, t);

    //normalizingSpeeds(v1.magnitude, v2.magnitude, v3.magnitude);

    v1 = angleOptimization(module1.measuredAngle, v1);
    v2 = angleOptimization(module2.measuredAngle - 120*DEG_TO_RAD, v2);
    v3 = angleOptimization(module3.measuredAngle + 120*DEG_TO_RAD, v3);

    // module1.moduleOffset();
    // module2.moduleOffset();
    // module3.moduleOffset();

}
void positionMode(Radio::Packet packet){

    float x = packet.tx - currentPosition.x;
    float y = packet.ty - currentPosition.y;
    float d = sqrtf(x*x +y*y);

    


    if (d < 1){
        t.x = 0.0f; //if you reach distance with tolerance, set speed to zero
        t.y = 0.0f;
    }
    else{
        t.x = x/d; //unit vectors for direction
        t.y = y/d;
    }
    float w = 0.0f; // change later pwitty pwease (don't forget stupido (if you miss this you big dummy))
    
    t = rotateVector(t, botAngle*DEG_TO_RAD);
    
    v1 = toWheelVelocity(w, module1, t);
    v2 = toWheelVelocity(w, module2, t);
    v3 = toWheelVelocity(w, module3, t);
    //normalizingSpeeds(v1.magnitude, v2.magnitude, v3.magnitude);

    v1 = angleOptimization(module1.measuredAngle, v1);
    v2 = angleOptimization(module2.measuredAngle - 120*DEG_TO_RAD, v2);
    v3 = angleOptimization(module3.measuredAngle + 120*DEG_TO_RAD, v3);

    static unsigned long lastCalc = millis();
    
    unsigned long timeIncrement = millis() - lastCalc;
    lastCalc = millis();
    if(timeIncrement > 1000){
        timeIncrement = 0;
    }

    testPrint = (float)timeIncrement;

    currentPosition.x += maxSpeed * t.x * (float)timeIncrement * 0.001;
    currentPosition.y += maxSpeed * t.y * (float)timeIncrement * 0.001;

    
    
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

        botAngle += (norm.YAxis) * GYRO_SAMPLE_TIME;
        

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