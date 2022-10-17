#include <SerialBridge.hpp>

#include <ThrowGain.h>

#define GAIN_MSG_ID 20

ThrowGain gain_msg;

//  Please init SB by yourself
SerialBridge serial = new SerialBridge();

int main()
{
  serial.add_frame(GAIN_MSG_ID, &gain_msg);

  gain_msg.data.gain = throw_gain;

  serial.write(GAIN_MSG_ID);
  
  return 0;
}
