#ifndef IMPORTANT_NUMBERS_H
#define IMPORTANT_NUMBERS_H

#include <Encoder.h>
#include <FastFloatPID.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SYSTEM CONFIGURATION

const uint8_t MODULE1_ADDRESS = 0x20,
              MODULE2_ADDRESS = 0x21,
              MODULE3_ADDRESS = 0x22;

const float ENCODER_TICKS_PER_REVOLUTION = 28;

const float STEERING_RATIO = 50.0 * 47.0 * 97.0 / 25.0 / 25.0 / 25.0,
            WHEEL_RATIO = STEERING_RATIO * 15.0 / 59.0;

const float WHEEL_CIRCUMFERENCE_IN = PI * 2.5;

const float SPEED_KP = 1.0,
            SPEED_KI = 0.0,
            SPEED_KD = 0.0,
            ANGLE_KP = 0.0,
            ANGLE_KI = 0.0,
            ANGLE_KD = 0.0;

#define CONTROLLER_I2C_PINS I2C_PINS_18_19      //must be a valid macro from the i2c_t3 library

Encoder mod1_m1_encoder(0,1),   mod1_m2_encoder(3,4);
//        mod2_m1_encoder(6,7),   mod2_m2_encoder(8,9),
//        mod3_m1_encoder(10,11), mod3_m2_encoder(12,13);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//PID inputs
float mod1_measuredspeed = 0, mod1_measuredangle = 0, 
      mod2_measuredspeed = 0, mod2_measuredangle = 0, 
      mod3_measuredspeed = 0, mod3_measuredangle = 0;

//PID setpoints
float mod1_targetspeed = 0, mod1_targetangle = 0, 
      mod2_targetspeed = 0, mod2_targetangle = 0, 
      mod3_targetspeed = 0, mod3_targetangle = 0;

//PID outputs
float mod1_PIDspeed = 0, mod1_PIDangle = 0, 
      mod2_PIDspeed = 0, mod2_PIDangle = 0, 
      mod3_PIDspeed = 0, mod3_PIDangle = 0;



boolean i2c_bus_error = false;

boolean module1_datawaiting = false,
        module2_datawaiting = false,
        module3_datawaiting = false;

//I2C data buffers
int8_t mod1_m1_buffervalue = 0, mod1_m2_buffervalue = 0,
       mod2_m1_buffervalue = 0, mod2_m2_buffervalue = 0, 
       mod3_m1_buffervalue = 0, mod3_m2_buffervalue = 0;



FastFloatPID mod1_speedctl(&mod1_measuredspeed, &mod1_PIDspeed, &mod1_targetspeed, SPEED_KP, SPEED_KI, SPEED_KD, REVERSE),
             mod1_anglectl(&mod1_measuredangle, &mod1_PIDangle, &mod1_targetangle, ANGLE_KP, ANGLE_KI, ANGLE_KD, REVERSE);
//             mod2_speedctl(&mod2_measuredspeed, &mod2_PIDspeed, &mod2_targetspeed, SPEED_KP, SPEED_KI, SPEED_KD, DIRECT),
//             mod2_anglectl(&mod2_measuredangle, &mod2_PIDangle, &mod2_targetangle, ANGLE_KP, ANGLE_KI, ANGLE_KD, DIRECT),
//             mod3_speedctl(&mod2_measuredspeed, &mod3_PIDspeed, &mod3_targetspeed, SPEED_KP, SPEED_KI, SPEED_KD, DIRECT),
//             mod3_anglectl(&mod2_measuredangle, &mod3_PIDangle, &mod3_targetangle, ANGLE_KP, ANGLE_KI, ANGLE_KD, DIRECT);

#endif