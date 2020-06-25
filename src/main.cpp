#include <Arduino.h>

#include <i2c_t3.h>
#include <Encoder.h>
#include <FastFloatPID.h>   //https://github.com/macaba/FastFloatPID

#include "important-numbers.h"
             
void updateMeasuredValues();
void formatAndSendPIDOutputs();
uint8_t homeModules();
void updateModuleController1(int8_t, int8_t);
void updateModuleController2(int8_t, int8_t);
void updateModuleController3(int8_t, int8_t);
void i2cTransmitCallback();


void setup() {
  Serial.begin(2000000);
  Serial.setTimeout(10000);
  #ifdef DEBUG_MODE
  while(!Serial);
  #endif
  
  Wire.begin(I2C_MASTER, 0x00, CONTROLLER_I2C_PINS, I2C_PULLUP_EXT, 100000);
  Wire.setDefaultTimeout(2000); // 2ms
  Wire.onTransmitDone(&i2cTransmitCallback);

  //homeModules(); //check return value?

  mod1_speedctl.SetSampleTime(0.001);
  mod1_anglectl.SetSampleTime(0.001);
//  mod2_speedctl.SetSampleTime(0.001);
//  mod2_anglectl.SetSampleTime(0.001);
//  mod3_speedctl.SetSampleTime(0.001);
//  mod3_anglectl.SetSampleTime(0.001);

mod1_speedctl.SetOutputLimits(-20, 20);
  mod1_anglectl.SetOutputLimits(-20, 20);
//  mod1_speedctl.SetOutputLimits(-127, 127);
//  mod1_anglectl.SetOutputLimits(-127, 127);
//  mod2_speedctl.SetOutputLimits(-127, 127);
//  mod2_anglectl.SetOutputLimits(-127, 127);
//  mod3_speedctl.SetOutputLimits(-127, 127);
//  mod3_anglectl.SetOutputLimits(-127, 127);
  
  mod1_speedctl.SetMode(AUTOMATIC);
  mod1_anglectl.SetMode(AUTOMATIC);
//  mod2_speedctl.SetMode(AUTOMATIC);
//  mod2_anglectl.SetMode(AUTOMATIC);
//  mod3_speedctl.SetMode(AUTOMATIC);
//  mod3_anglectl.SetMode(AUTOMATIC);
}

void loop() {
//  if(Serial.available() > 6) {
//    String b = Serial.readStringUntil('\n');
//    Serial.println(b.substring(0,3).toInt());
//    Serial.println(b.substring(4,7).toInt());
//    updateModuleController1(b.substring(0,3).toInt(), b.substring(4,7).toInt());
//  }

//updateModuleController1(20, 20);
//delay(1200);
//updateModuleController1(0, 0);
//delay(1200);
//updateModuleController1(-20, -20);
//delay(1200);
//updateModuleController1(0, 0);
//delay(1200);

  static unsigned long lastPIDcalc = 0;
  if(Serial.available() > 6) {
    String b = Serial.readStringUntil('\n');
    
    mod1_targetspeed = b.substring(0,4).toFloat();
    mod1_targetangle = DEG_TO_RAD * b.substring(5,8).toFloat();
  }


  if(micros() - lastPIDcalc > 1000) {     //this syntax still works through a timer overflow
    updateMeasuredValues();
    
    mod1_speedctl.Compute();
    mod1_anglectl.Compute();
//    mod2_speedctl.Compute();
//    mod2_anglectl.Compute();
//    mod3_speedctl.Compute();
//    mod3_anglectl.Compute();

    formatAndSendPIDOutputs();
    lastPIDcalc = micros();
  }
}



void updateMeasuredValues() {
  static unsigned long lastCalcTime = micros();
  int32_t mod1_m1_ticks = mod1_m1_encoder.read(),
          mod1_m2_ticks = mod1_m2_encoder.read();
//          mod2_m1_ticks = mod2_m1_encoder.read(),
//          mod2_m2_ticks = mod2_m2_encoder.read(),
//          mod3_m1_ticks = mod3_m1_encoder.read(),
//          mod3_m2_ticks = mod3_m2_encoder.read();
  static int32_t last_mod1_dist = 0;
//                 last_mod2_dist = 0,
//                 last_mod3_dist = 0;

                 
  if(lastCalcTime > micros()) lastCalcTime = 0;   //overflow mitigation
  float deltaT = micros() - lastCalcTime;
  if(deltaT < 250) deltaT = 1000;

  float mod1_dist = (WHEEL_CIRCUMFERENCE_IN * (mod1_m1_ticks - mod1_m2_ticks)) / (ENCODER_TICKS_PER_REVOLUTION * WHEEL_RATIO); 
  mod1_measuredspeed = (1e6 * (mod1_dist - last_mod1_dist)) / deltaT;

  mod1_measuredangle = (PI *(mod1_m1_ticks + mod1_m2_ticks)) / (ENCODER_TICKS_PER_REVOLUTION * STEERING_RATIO);   //the 2 from the radian conversion and the average calculation cancel out

  
  //save encoder values for next calculation
  lastCalcTime = micros();
  last_mod1_dist = mod1_dist;
}

void formatAndSendPIDOutputs() {
  int m1_out = mod1_PIDspeed;
  int m2_out = -mod1_PIDspeed;
  m1_out += mod1_PIDangle;
  m2_out += mod1_PIDangle;

  //normalize
  if(abs(m1_out) > 127 || abs(m2_out) > 127) {
    if(abs(m1_out) > abs(m2_out)) {
      m2_out = (127 * m2_out) / abs(m1_out); 
      m1_out = (127 * m1_out) / abs(m1_out);
    }
    else {
      m1_out = (127 * m1_out) / abs(m2_out); 
      m2_out = (127 * m2_out) / abs(m2_out);
    }
  }
  if(i2c_bus_error) {
    Serial.println("error");
    Serial.println(Wire.getError());
  }
  updateModuleController1((int8_t)m1_out, (int8_t)m2_out);
}

//uint8_t homeModules() {
//  unsigned long startTime = millis();
//  boolean homingError = false;
//  while(1) {
//    updateModuleController1(5, 5);
//    
//    if(/*homing success*/0) {
//      for(byte i = 0; i < 10; i++) updateModuleController1(0, 0);
//      mod1_m1_encoder.write(0);
//      mod1_m2_encoder.write(0);
//      break;
//    }
//    if(millis() - startTime > 5000) {
//      for(byte i = 0; i < 10; i++) updateModuleController1(0, 0);
//      homingError = true;
//      break;
//    }
//  }
//  startTime = millis();
//  while(1) {
//    updateModuleController2(5, 5);
//    
//    if(/*homing success*/0) {
//      for(byte i = 0; i < 10; i++) updateModuleController2(0, 0);
//      mod2_m1_encoder.write(0);
//      mod2_m2_encoder.write(0);
//      break;
//    }
//    if(millis() - startTime > 5000) {
//      for(byte i = 0; i < 10; i++) updateModuleController2(0, 0);
//      homingError = true;
//      break;
//    }
//  }
//  startTime = millis();
//  while(1) {
//    updateModuleController3(5, 5);
//    
//    if(/*homing success*/0) {
//      for(byte i = 0; i < 10; i++) updateModuleController3(0, 0);
//      mod3_m1_encoder.write(0);
//      mod3_m2_encoder.write(0);
//      break;
//    }
//    if(millis() - startTime > 5000) {
//      for(byte i = 0; i < 10; i++) updateModuleController3(0, 0);
//      homingError = true;
//      break;
//    }
//  }
//  if(homingError) return 1;
//  else return 0;
//}


void updateModuleController1(int8_t m1, int8_t m2) {
  if(Wire.done()) {
    Wire.beginTransmission(MODULE1_ADDRESS);
    Wire.write(m1);
    Wire.write(m2);
    Wire.sendTransmission();
    if(Wire.getError()) i2c_bus_error = true;
  } 
  else {
    module1_datawaiting = true;
    mod1_m1_buffervalue = m1;
    mod1_m2_buffervalue = m2;
  }
}

//
//void updateModuleController2(int8_t m1, int8_t m2) {
//  if(Wire.done()) {
//    Wire.beginTransmission(MODULE2_ADDRESS);
//    Wire.write(m1);
//    Wire.write(m2);
//    Wire.sendTransmission();
//    if(Wire.getError()) i2c_bus_error = true;
//  } 
//  else {
//    module2_datawaiting = true;
//    mod2_m1_buffervalue = m1;
//    mod2_m2_buffervalue = m2;
//  }
//}
//
//
//void updateModuleController3(int8_t m1, int8_t m2) {
//  if(Wire.done()) {
//    Wire.beginTransmission(MODULE3_ADDRESS);
//    Wire.write(m1);
//    Wire.write(m2);
//    Wire.sendTransmission();
//    if(Wire.getError()) i2c_bus_error = true;
//  } 
//  else {
//    module3_datawaiting = true;
//    mod3_m1_buffervalue = m1;
//    mod3_m2_buffervalue = m2;
//  }
//}

void i2cTransmitCallback() {
  if(Wire.getError()) {
    Wire.resetBus();
    i2c_bus_error = true;
  }
  
  if(module1_datawaiting) {
    Wire.beginTransmission(MODULE1_ADDRESS);
    Wire.write(mod1_m1_buffervalue);
    Wire.write(mod1_m2_buffervalue);
    Wire.sendTransmission();
    module1_datawaiting = false;
  } 
  else if(module2_datawaiting) {
    Wire.beginTransmission(MODULE1_ADDRESS);
    Wire.write(mod2_m1_buffervalue);
    Wire.write(mod2_m2_buffervalue);
    Wire.sendTransmission();
    module2_datawaiting = false;
  } 
  else if(module3_datawaiting) {
    Wire.beginTransmission(MODULE3_ADDRESS);
    Wire.write(mod3_m1_buffervalue);
    Wire.write(mod3_m2_buffervalue);
    Wire.sendTransmission();
    module3_datawaiting = false;
  }
}