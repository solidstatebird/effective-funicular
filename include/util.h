#ifndef UTIL_H
#define UTIL_H

#include <inttypes.h>

class MovingAverage
{
public:
    void add(float);
    float average(void);
    void zero(void);

private:
#define DATA_SIZE 5
    uint_fast8_t index = 0;
    float data[DATA_SIZE];
};

#endif