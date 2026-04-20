#include <iostream>

#define JITBUS_DISABLE_LOG
#include <jitbus.h>

SerialJitbus jit;

struct Imu {
    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    float linear_acceleration_x;
    float linear_acceleration_y;
    float linear_acceleration_z;
};

Imu imu; // IMU struct message, has to be the same on the sender
int imu_id = 0; // IMU identificator, has to be the same on the sender

bool new_data = false;

int main(void)
{

  if (jit.init("/dev/ttyUSB0", 115200))
  {
    std::cout << "Serial port opened! \n";
  }

  while(true)
  {
    if (jit.available() > 0)
    {
      if (jit.receivePacket(imu, imu_id))
      {
        std::cout << "------------\n"; 
        std::cout << "x = " << imu.orientation_x << "\n";
        std::cout << "y = " << imu.orientation_y << "\n";
        std::cout << "z = " << imu.orientation_z << "\n";
        std::cout << "w = " << imu.orientation_z<< "\n";
      }
    }
  }

  return 0;
}