#ifndef _PID_GAIN_HPP_
#define _PID_GAIN_HPP_

#include <Message.hpp>
#include "MessageStructure.hpp"

typedef struct
{
  float kp;
  float ki;
  float kd;
  float fg;
} gain_t;

typedef struct PIDGainType
{
  gain_t gains[4];
} pid_gain_t;

//  create message
typedef sb::Message<pid_gain_t> PIDGain;

#endif