#include <Arduino.h>

#include <Encoder.h>
#include <FastFloatPID.h>     //https://github.com/macaba/FastFloatPID

#include "radio.h"
#include "important-numbers.h"

//#include "util.h"

void updateMeasuredValues();
void updateModuleController(int, int, uint8_t);
void formatAndSendPIDOutputs();
uint8_t homeModules();


void setup() {
    Serial.begin(2000000);
    Serial.setTimeout(10000);

    radioInitialize();

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
    // mod1_targetspeed = 30;
    // static unsigned long lastANGLECHANGE = 0;
    // if(millis() - lastANGLECHANGE > 1000) {
    //     // mod1_targetangle = 3.14 * sin(millis() * 0.00328);
    //     // mod1_targetspeed = 60 * sin(HALF_PI/2 + millis() * 0.00328);
    //     mod1_targetangle += (6.0 * DEG_TO_RAD);
    //     lastANGLECHANGE = millis();
    // }

       

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
    }
    
    static unsigned long lastSpeedCalc = 0;
    if(micros() - lastSpeedCalc > 1e6 * SPEED_PID_SAMPLE_TIME) {         //this syntax still works through a timer overflow
        static int32_t lastPosition = 0;
        int32_t position = mod1_m1_encoder.read() - mod1_m2_encoder.read();
        
        mod1_measuredspeed = (WHEEL_CIRCUMFERENCE_IN * (position - lastPosition))
                                / ( ENCODER_TICKS_PER_REVOLUTION * WHEEL_RATIO * SPEED_PID_SAMPLE_TIME);
 
        mod1_speedctl.Compute();
        mod2_speedctl.Compute();
        mod3_speedctl.Compute();

        formatAndSendPIDOutputs();

        lastPosition = position;
        lastSpeedCalc = micros();
    }

    radioUpdate();

    if(radioPacketAvailable()) {
        Packet packet = radioGetPacket(); 
        if(GETFLAG(packet.flags, FLAG_ENABLE)) {
            if(!enabled) {
                mod1_anglectl.SetMode(AUTOMATIC);
                mod2_anglectl.SetMode(AUTOMATIC);
                mod3_anglectl.SetMode(AUTOMATIC);
                mod1_speedctl.SetMode(AUTOMATIC);
                mod2_speedctl.SetMode(AUTOMATIC);
                mod3_speedctl.SetMode(AUTOMATIC);
                enabled = true;
            }
        }

        if(GETFLAG(packet.flags, FLAG_ACK)) {
            radioSendStatus();
        }

        if(enabled) {
            mod1_targetspeed = packet.s1;
            mod2_targetspeed = packet.s2;
            mod3_targetspeed = packet.s3;
            mod1_targetangle = packet.a1;
            mod2_targetangle = packet.a2;
            mod3_targetangle = packet.a3;
            
            //prevent the speed PID from sending an output in the wrong direction
            if(mod1_targetspeed >= 0) {
                mod1_speedctl.SetOutputLimits(0, MAX_MOTOR_OUTPUT);
            } else {
                mod1_speedctl.SetOutputLimits(-MAX_MOTOR_OUTPUT, 0);
            }
            //clear any I error accumulation when desired speed is zero
            if(mod1_targetspeed == 0) {
                mod1_speedctl.SetMode(MANUAL);
                mod1_speedctl.SetMode(AUTOMATIC);
            } 
            if(mod2_targetspeed >= 0) {
                mod2_speedctl.SetOutputLimits(0, MAX_MOTOR_OUTPUT);
            } else {
                mod2_speedctl.SetOutputLimits(-MAX_MOTOR_OUTPUT, 0);
            }
            if(mod2_targetspeed == 0) {
                mod2_speedctl.SetMode(MANUAL);
                mod2_speedctl.SetMode(AUTOMATIC);
            } 
            if(mod3_targetspeed >= 0) {
                mod3_speedctl.SetOutputLimits(0, MAX_MOTOR_OUTPUT);
            } else {
                mod3_speedctl.SetOutputLimits(-MAX_MOTOR_OUTPUT, 0);
            }
            if(mod3_targetspeed == 0) {
                mod3_speedctl.SetMode(MANUAL);
                mod3_speedctl.SetMode(AUTOMATIC);
            } 
        }
    }

    static unsigned long disableTimer = millis();
    if(millis() - disableTimer > SAFETY_TIMEOUT_MS) {
        mod1_targetspeed = 0;
        mod2_targetspeed = 0;
        mod3_targetspeed = 0;
        mod1_anglectl.SetMode(MANUAL);
        mod2_anglectl.SetMode(MANUAL);
        mod3_anglectl.SetMode(MANUAL);
        mod1_speedctl.SetMode(MANUAL);
        mod2_speedctl.SetMode(MANUAL);
        mod3_speedctl.SetMode(MANUAL);
        mod1_targetspeed = 0;
        mod2_targetspeed = 0;
        mod3_targetspeed = 0;
        mod1_PIDspeed = 0;
        mod2_PIDspeed = 0;
        mod3_PIDspeed = 0;
        mod1_PIDangle = 0;
        mod2_PIDangle = 0;
        mod3_PIDangle = 0;
        enabled = false;
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

uint8_t homeModules() {
    unsigned long startTime = millis();
    boolean homingError = false;
    while(1) {
        updateModuleController(40, 40, MODULE_1);
        
        if(analogRead(MOD3_HALLPIN) < MAGNET_THRESHOLD) {
            updateModuleController(0, 0, MODULE_1);
            break;
        }
        if(millis() - startTime > 5000) {
            updateModuleController(0, 0, MODULE_1);
            homingError = true;
            break;
        }
    }
    mod1_m1_encoder.write(0);
    mod1_m2_encoder.write(0);

    // startTime = millis();

    // while(1) {
    //     updateModuleController(40, 40, MODULE_2);
        
    //     if(analogRead(MOD2_HALLPIN) < MAGNET_THRESHOLD)  {
    //         updateModuleController(0, 0, MODULE_2);
    //         break;
    //     }
    //     if(millis() - startTime > 5000) {
    //         updateModuleController(0, 0, MODULE_2);
    //         homingError = true;
    //         break;
    //     }
    // }
    // mod2_m1_encoder.write(0);
    // mod2_m2_encoder.write(0);

    // startTime = millis();

    // while(1) {
    //     updateModuleController(40, 40, MODULE_3);
        
    //     if(analogRead(MOD3_HALLPIN) < MAGNET_THRESHOLD)  {
    //         updateModuleController(0, 0, MODULE_3);
    //         break;
    //     }
    //     if(millis() - startTime > 5000) {
    //         updateModuleController(0, 0, MODULE_3);
    //         homingError = true;
    //         break;
    //     }
    // }
    // mod3_m1_encoder.write(0);
    // mod3_m2_encoder.write(0);

    if(homingError) return 1;
    else return 0;
}


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