#include <iostream>
#include <chrono>

#define JITBUS_DISABLE_LOG
#include <jitbus.h>

SerialJitbus jit;

float motor_speed[2]{2.5,2.5};  // Motor speed float array message, has to be the same on the receiver
int motor_speed_id = 2;         // Motor package identificator, has to be the same on the receiver
uint32_t motor_speed_timer = 0; // Auxiliar variable required for sendPacketHz()

// Variables for toggling motor speed data
const auto period = std::chrono::milliseconds(5000); 
auto last = std::chrono::steady_clock::now();

int main(void)
{
  
  if (jit.init("/dev/ttyUSB0", 115200))
  {
    std::cout << "Serial port opened! \n";
  }

  while(true)
  {
    // Send data at 50 hz. Requires polling for sending
    jit.sendPacketHz(motor_speed, motor_speed_id, motor_speed_timer, 50);

    // Send data, non-blocking. Requires polling for sending
    // jit.sendPacket(motor_speed, motor_speed_id);

    // Send data, blocking.
    //jit.sendPacketBlocking(motor_speed, motor_speed_id);

    // Toggle speed every 5 seconds
    if (std::chrono::steady_clock::now() - last >= period)
    {
      if (motor_speed[0] != 0 && motor_speed[1] != 0)
      {
        std::cout << "Stop motors\n";
        motor_speed[0] = 0.0;
        motor_speed[1] = 0.0;
      }
      else
      {
        std::cout << "Move motors\n";
        motor_speed[0] = 2.5;
        motor_speed[1] = 2.5;
      }
      last += period;
    }
  }

  return 0;
}