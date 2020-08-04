#include "util.h"

MovingAverage::MovingAverage(uint8_t size_)
    : size(size_)
{
    if (size == 0)
        size = 1;
    data = new float[size];
}

MovingAverage::~MovingAverage()
{
    delete[] data;
}

void MovingAverage::add(float value)
{
    data[index] = value;
    index++;
    if (index > (size - 1))
        index = 0;
}

float MovingAverage::average()
{
    float output = 0;
    for (int_fast8_t i = 0; i < size; i++)
        output += data[i];
    return output / size;
}

void MovingAverage::zero()
{
    for (int_fast8_t i = 0; i < size; i++)
        data[i] = 0;
}