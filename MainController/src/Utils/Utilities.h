#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>

// Clamp a value between min and max
inline float clamp(float value, float minVal, float maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

// Map a float from one range to another
inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif // UTILITIES_H
