#include "util.h"

void MovingAverage::add(float value) {
    data[index] = value;
    index++;
    if(index > (DATA_SIZE - 1)) index = 0;
}

float MovingAverage::average() {
    float output = 0;
    for(int_fast8_t i = 0; i < DATA_SIZE; i++) output += data[i];
    return output / DATA_SIZE; 
}

void MovingAverage::zero() {
    for(int_fast8_t i = 0; i < DATA_SIZE; i++) data[i] = 0;
}