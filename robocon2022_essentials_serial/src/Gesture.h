#ifndef GESTURE_H_
#define GESTURE_H_

#include <Message.hpp>

typedef struct GestureType
{
    int type;
} gesture_t;

//  create message
typedef sb::Message<gesture_t> Gesture;

#endif