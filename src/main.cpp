#include <Arduino.h>

#include <Encoder.h>
#include <FastFloatPID.h>     //https://github.com/macaba/FastFloatPID

#include "important-numbers.h"
#include "util.h"

void updateMeasuredValues();
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
        mod1_speedctl.SetSampleTime(0.002);
        mod1_anglectl.SetSampleTime(0.002);
    //    mod2_speedctl.SetSampleTime(0.002);
    //    mod2_anglectl.SetSampleTime(0.002);
    //    mod3_speedctl.SetSampleTime(0.002);
    //    mod3_anglectl.SetSampleTime(0.002);

        mod1_speedctl.SetOutputLimits(-255, 255);
        mod1_anglectl.SetOutputLimits(-255, 255);
    //    mod2_speedctl.SetOutputLimits(-255, 255);
    //    mod2_anglectl.SetOutputLimits(-255, 255);
    //    mod3_speedctl.SetOutputLimits(-255, 255);
    //    mod3_anglectl.SetOutputLimits(-255, 255);
        
    //    mod1_speedctl.SetMode(AUTOMATIC);
        mod1_anglectl.SetMode(AUTOMATIC);
    //    mod2_speedctl.SetMode(AUTOMATIC);
    //    mod2_anglectl.SetMode(AUTOMATIC);
    //    mod3_speedctl.SetMode(AUTOMATIC);
    //    mod3_anglectl.SetMode(AUTOMATIC);
    }
      
}

void loop() {
//    if(Serial.available() > 6) {
//        String b = Serial.readStringUntil('\n');
//        Serial.println(b.substring(0,3).toInt());
//        Serial.println(b.substring(4,7).toInt());
//        updateModuleController1(b.substring(0,3).toInt(), b.substring(4,7).toInt());
//    }

// updateModuleController1(20, 20);
// delay(1200);
// updateModuleController1(0, 0);
// delay(1200);
// updateModuleController1(-20, -20);
// delay(1200);
// updateModuleController1(0, 0);
// delay(1200);

    static unsigned long lastPIDcalc = 0;
    
    if(Serial.available() > 6) {
        String b = Serial.readStringUntil('\n');
        
        mod1_targetspeed = b.substring(0,4).toFloat();
        mod1_targetangle = DEG_TO_RAD * b.substring(5,8).toFloat();
    }

    if(micros() - lastPIDcalc > 2000) {         //this syntax still works through a timer overflow
        updateMeasuredValues();
        
        // mod1_speedctl.Compute();
        mod1_anglectl.Compute();
        // mod2_speedctl.Compute();
        // mod2_anglectl.Compute();
        // mod3_speedctl.Compute();
        // mod3_anglectl.Compute();

        formatAndSendPIDOutputs();
        lastPIDcalc = micros();
    }
}



void updateMeasuredValues() {
    int32_t mod1_m1_ticks = mod1_m1_encoder.read(),
            mod1_m2_ticks = mod1_m2_encoder.read();

    mod1_measuredangle = (PI *(mod1_m1_ticks + mod1_m2_ticks)) / (ENCODER_TICKS_PER_REVOLUTION * STEERING_RATIO);     //the 2 from the radian conversion and the average calculation cancel out
}

void formatAndSendPIDOutputs() {
    //module 1
    {
        int m1_out = (127 * mod1_targetspeed / 327);
        int m2_out = -(127 * mod1_targetspeed / 327);
        m1_out += mod1_PIDangle;
        m2_out += mod1_PIDangle;

        //normalize
        if(abs(m1_out) > 255 || abs(m2_out) > 255) {
            if(abs(m1_out) > abs(m2_out)) {
                m2_out = (255 * m2_out) / abs(m1_out); 
                m1_out = (255 * m1_out) / abs(m1_out);
            }
            else {
                m1_out = (255 * m1_out) / abs(m2_out); 
                m2_out = (255 * m2_out) / abs(m2_out);
            }
        }

        if(m1_out >= 0) {
            digitalWrite(MOD1_M1_DIRPIN, LOW);
            analogWrite(MOD1_M1_PWMPIN, m1_out);
        } 
        else {
            digitalWrite(MOD1_M1_DIRPIN, HIGH);
            analogWrite(MOD1_M1_PWMPIN, abs(m1_out));
        }
        if(m2_out >= 0) {
            digitalWrite(MOD1_M2_DIRPIN, LOW);
            analogWrite(MOD1_M2_PWMPIN, m2_out);
        } 
        else {
            digitalWrite(MOD1_M2_DIRPIN, HIGH);
            analogWrite(MOD1_M2_PWMPIN, abs(m2_out));
    }
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
