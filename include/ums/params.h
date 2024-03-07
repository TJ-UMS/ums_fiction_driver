#include <cstdint>
#ifndef PARAMS_H
#define PARAMS_H

struct ParamsData
{
    float KP;
    float KI;
    float KD;
    float LA;
    float LB;
    int32_t KMTT;
    float IMU_Z;
};

#endif