#include <Arduino.h>

#include <Encoder.h>
#include <FastFloatPID.h>     //https://github.com/macaba/FastFloatPID

#include "important-numbers.h"
//#include "util.h"

#define MAX_MOTOR_OUTPUT 255

void updateMeasuredValues();
void updateModuleController(int, int, uint8_t);
void formatAndSendPIDOutputs();
uint8_t homeModules();


void setup() {
    Serial.begin(2000000);
    Serial.setTimeout(10000);

    //pin setup
    {
        pinMode(MOD1_M1_DIRPIN, OUTPUT); pinMode(MOD1_M2_DIRPIN, OUTPUT);
        pinMode(MOD2_M1_DIRPIN, OUTPUT); pinMode(MOD2_M2_DIRPIN, OUTPUT);
        pinMode(MOD3_M1_DIRPIN, OUTPUT); pinMode(MOD3_M2_DIRPIN, OUTPUT);
        analogWriteFrequency(MOD1_M1_DIRPIN, 15000); analogWriteFrequency(MOD1_M2_DIRPIN, 15000);
        analogWriteFrequency(MOD2_M1_DIRPIN, 15000); analogWriteFrequency(MOD2_M2_DIRPIN, 15000);
        analogWriteFrequency(MOD3_M1_DIRPIN, 15000); analogWriteFrequency(MOD3_M2_DIRPIN, 15000);
    }

    //homeModules(); //check return value?

    //PID setup
    {
        mod1_anglectl.SetSampleTime(ANGLE_PID_SAMPLE_TIME);
        mod2_anglectl.SetSampleTime(ANGLE_PID_SAMPLE_TIME);
        mod3_anglectl.SetSampleTime(ANGLE_PID_SAMPLE_TIME);

        mod1_speedctl.SetSampleTime(SPEED_PID_SAMPLE_TIME);
        mod2_speedctl.SetSampleTime(SPEED_PID_SAMPLE_TIME);
        mod3_speedctl.SetSampleTime(SPEED_PID_SAMPLE_TIME);

        mod1_speedctl.SetOutputLimits(-MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
        mod2_speedctl.SetOutputLimits(-MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
        mod3_speedctl.SetOutputLimits(-MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);

        mod1_anglectl.SetOutputLimits(-MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
        mod2_anglectl.SetOutputLimits(-MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
        mod3_anglectl.SetOutputLimits(-MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
        
        mod1_speedctl.SetMode(AUTOMATIC);
    //    mod2_speedctl.SetMode(AUTOMATIC);
    //    mod3_speedctl.SetMode(AUTOMATIC);

        mod1_anglectl.SetMode(AUTOMATIC);
    //    mod2_anglectl.SetMode(AUTOMATIC);
    //    mod3_anglectl.SetMode(AUTOMATIC);
    }
      
}

void loop() {
    // mod1_targetspeed = 60;
    // static unsigned long lastANGLECHANGE = 0;
    // if(millis() - lastANGLECHANGE > 700) {
    //     // mod1_targetangle = 3.14 * sin(millis() * 0.00328);
    //     // mod1_targetspeed = 60 * sin(HALF_PI/2 + millis() * 0.00328);
    //     mod1_targetangle += (90.0 * DEG_TO_RAD);
    //     lastANGLECHANGE = millis();
    // }

    
    if(Serial.available() > 6) {
        String b = Serial.readStringUntil('\n');
        
        mod1_targetspeed = b.substring(0,4).toFloat();
        mod1_targetangle = DEG_TO_RAD * b.substring(5,8).toFloat();
    }

    static unsigned long lastAngleCalc = 0;
    if(micros() - lastAngleCalc > 1e6 * ANGLE_PID_SAMPLE_TIME) {         //this syntax still works through a timer overflow
        mod1_measuredangle = (PI * (mod1_m1_encoder.read() + mod1_m2_encoder.read()))
                                / (ENCODER_TICKS_PER_REVOLUTION * STEERING_RATIO);    
                                         //the 2 from the radian conversion and the averaging calculation cancel out
        
        mod1_anglectl.Compute();
        mod2_anglectl.Compute();
        mod3_anglectl.Compute();

        formatAndSendPIDOutputs();

        lastAngleCalc = micros();

        Serial.print(mod1_measuredspeed);Serial.print("     ");
        Serial.println(mod1_PIDspeed);
        // Serial.println(mod1_m1_encoder.read());
        // Serial.println(mod1_m2_encoder.read());
    }
    
    static unsigned long lastSpeedCalc = 0;
    if(micros() - lastSpeedCalc > 1e6 * SPEED_PID_SAMPLE_TIME) {         //this syntax still works through a timer overflow
        static int32_t lastPosition = 0;
        int32_t position = mod1_m1_encoder.read() - mod1_m2_encoder.read();
        
        mod1_measuredspeed = (WHEEL_CIRCUMFERENCE_IN * (position - lastPosition))
                                / ( ENCODER_TICKS_PER_REVOLUTION * WHEEL_RATIO * SPEED_PID_SAMPLE_TIME);
        
        //prevent the speed PID from sending an output in the wrong direction
        if(mod1_targetspeed >= 0) {
            mod1_speedctl.SetOutputLimits(0, MAX_MOTOR_OUTPUT);
        } else {
            mod1_speedctl.SetOutputLimits(-MAX_MOTOR_OUTPUT, 0);
        }
        

        mod1_speedctl.Compute();
        mod2_speedctl.Compute();
        mod3_speedctl.Compute();

        formatAndSendPIDOutputs();

        lastPosition = position;
        lastSpeedCalc = micros();
    }
}


void formatAndSendPIDOutputs() {
    //module 1
    {
        int m1_out = -mod1_PIDspeed;
        int m2_out = mod1_PIDspeed;
        m1_out += mod1_PIDangle;
        m2_out += mod1_PIDangle;

        //normalize
        if(abs(m1_out) > MAX_MOTOR_OUTPUT || abs(m2_out) > MAX_MOTOR_OUTPUT) {
            if(abs(m1_out) > abs(m2_out)) {
                m2_out = (MAX_MOTOR_OUTPUT * m2_out) / abs(m1_out); 
                m1_out = (MAX_MOTOR_OUTPUT * m1_out) / abs(m1_out);
            }
            else {
                m1_out = (MAX_MOTOR_OUTPUT * m1_out) / abs(m2_out); 
                m2_out = (MAX_MOTOR_OUTPUT * m2_out) / abs(m2_out);
            }
        }

        updateModuleController(m1_out, m2_out, MODULE_1);
    }
}

//uint8_t homeModules() {
//    unsigned long startTime = millis();
//    boolean homingError = false;
//    while(1) {
//        updateModuleController1(5, 5);
//        
//        if(/*homing success*/0) {
//            for(byte i = 0; i < 10; i++) updateModuleController1(0, 0);
//            mod1_m1_encoder.write(0);
//            mod1_m2_encoder.write(0);
//            break;
//        }
//        if(millis() - startTime > 5000) {
//            for(byte i = 0; i < 10; i++) updateModuleController1(0, 0);
//            homingError = true;
//            break;
//        }
//    }
//    startTime = millis();
//    while(1) {
//        updateModuleController2(5, 5);
//        
//        if(/*homing success*/0) {
//            for(byte i = 0; i < 10; i++) updateModuleController2(0, 0);
//            mod2_m1_encoder.write(0);
//            mod2_m2_encoder.write(0);
//            break;
//        }
//        if(millis() - startTime > 5000) {
//            for(byte i = 0; i < 10; i++) updateModuleController2(0, 0);
//            homingError = true;
//            break;
//        }
//    }
//    startTime = millis();
//    while(1) {
//        updateModuleController3(5, 5);
//        
//        if(/*homing success*/0) {
//            for(byte i = 0; i < 10; i++) updateModuleController3(0, 0);
//            mod3_m1_encoder.write(0);
//            mod3_m2_encoder.write(0);
//            break;
//        }
//        if(millis() - startTime > 5000) {
//            for(byte i = 0; i < 10; i++) updateModuleController3(0, 0);
//            homingError = true;
//            break;
//        }
//    }
//    if(homingError) return 1;
//    else return 0;
//}


void updateModuleController(int m1, int m2, uint8_t selector) {
    uint8_t dirpin1;
    uint8_t pwmpin1;
    uint8_t dirpin2;
    uint8_t pwmpin2;

    if(selector == MODULE_1) {
        dirpin1 = MOD1_M1_DIRPIN;
        dirpin2 = MOD1_M2_DIRPIN;
        pwmpin1 = MOD1_M1_PWMPIN;
        pwmpin2 = MOD1_M2_PWMPIN;
    }
    else if(selector == MODULE_2) {
        dirpin1 = MOD2_M1_DIRPIN;
        dirpin2 = MOD2_M2_DIRPIN;
        pwmpin1 = MOD2_M1_PWMPIN;
        pwmpin2 = MOD2_M2_PWMPIN;
    }
    else if(selector == MODULE_3) {
        dirpin1 = MOD3_M1_DIRPIN;
        dirpin2 = MOD3_M2_DIRPIN;
        pwmpin1 = MOD3_M1_PWMPIN;
        pwmpin2 = MOD3_M2_PWMPIN;
    }
    else return;

    if(m1 >= 0) {
        digitalWrite(dirpin1, LOW);
        analogWrite(pwmpin1, m1);
    } 
    else {
        digitalWrite(dirpin1, HIGH);
        analogWrite(pwmpin1, abs(m1));
    }
    if(m2 >= 0) {
        digitalWrite(dirpin2, LOW);
        analogWrite(pwmpin2, m2);
    } 
    else {
        digitalWrite(dirpin2, HIGH);
        analogWrite(pwmpin2, abs(m2));
    }
}