#ifndef _MESSAGE_STRUCTURE_HPP_
#define _MESSAGE_STRUCTURE_HPP_

#include "stdint.h"

typedef struct ShooterType
{
    //  shooter num
    int8_t num;
    //  Range 0-1
    float power;
    /**
     *  0: nothing
     *  1: up
     *  2: down
     *  3: shoot
     */
    int8_t action;
} shooter_t;

typedef struct Vector3Type
{
    float x;
    float y;
    float z;
} vector3_t;

typedef struct Vector4Type
{
    float v1;
    float v2;
    float v3;
    float v4;
} vector4_t;

#endif
