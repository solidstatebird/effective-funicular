#ifndef UTIL_H
#define UTIL_H

#include <inttypes.h>

class MovingAverage
{
public:
    MovingAverage(uint8_t);
    ~MovingAverage();
    void add(float);
    float average(void);
    void zero(void);

private:
    uint8_t size;
    uint8_t index = 0;
    float *data;
};

#endif