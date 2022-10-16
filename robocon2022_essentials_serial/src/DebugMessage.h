#ifndef DEBUG_MESSAGE_H_
#define DEBUG_MESSAGE_H_

#include <Message.hpp>

typedef struct DebugMessageType
{
    char str[64];
} debug_message_t;

//  create message
typedef sb::Message<debug_message_t> DebugMessage;

#endif