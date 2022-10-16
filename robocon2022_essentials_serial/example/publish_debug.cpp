#include <SerialBridge.hpp>

#include <stdio.h>
#include <DebugMessage.h>

#define DEBUG_MSG_ID 15

DebugMessage debug_msg;

//  Please init SB by yourself
SerialBridge serial = new SerialBridge();

int main()
{
  serial.add_frame(DEBUG_MSG_ID, &debug_msg);

  //  printf
  sprintf(
    debug_msg.data.str,
    "int: %d",
    123456
    );
  
  serial.write(DEBUG_MSG_ID);
  
  return 0;
}
