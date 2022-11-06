#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_

#include <Message.hpp>
#include "MessageStructure.hpp"

typedef struct ControllerType
{
    //  include vector-x,y,theta
    vector3_t movement;
    //  shooter variable
    bool all_reload;
    shooter_t shooter;
} controller_t;

//  create message
typedef sb::Message<controller_t> Controller;

#endif