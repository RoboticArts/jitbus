#include "ros/ros.h"
#include "jitbus.h"

struct Twist {
  uint16_t pos_x;
  uint16_t pos_y;
  uint16_t pos_z;
};

Twist twist;

struct Imu {
  float acc_x;
  uint32_t acc_y;
  float acc_z;
};

Imu imu;

SerialJitbus jit;
enum id_list {twist_id, odom_id, imu_id};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "serial_jitbus_test");
	ros::NodeHandle n;

	jit.init("/dev/ttyACM0", 9600);

	ros::Rate loop_rate(2);

	while (ros::ok()){

		//if (jit.connected()){

			jit.init("/dev/ttyACM0", 9600);

			/*
			if (jit.available()){

				jit.receivePacket(twist, 66);
				//printf("%d %d %d\n", twist.pos_x, twist.pos_y, twist.pos_z);

				jit.receivePacket(imu, 66);
				//printf("%f %d %f\n", imu.acc_x, imu.acc_y, imu.acc_z);

			}
			*/
		//}

		ros::spinOnce();
		loop_rate.sleep();
	}

}