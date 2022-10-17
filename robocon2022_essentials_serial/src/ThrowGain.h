#ifndef THROW_GAIN_H_
#define THROW_GAIN_H_

#include <Message.hpp>

typedef struct ThrowGainType
{
    float gain;
} throw_gain_t;

//  create message
typedef sb::Message<throw_gain_t> ThrowGain;

#endif