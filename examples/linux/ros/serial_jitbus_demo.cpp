#include "ros/ros.h"
#include "jitbus.h"

SerialJitbus jit;
double last_time = 0;
double last_time_debug = 0;

// --------- Received packets ------ //

struct Encoder {
  uint32_t seq;
  uint32_t left_encoder;
  uint32_t right_encoder;

};

Encoder encoder;
int encoder_id = 77;

struct Sensor {
  uint32_t seq;
  uint16_t left_sensor;
  uint16_t right_sensor;
};

Sensor sensor;
int sensor_id = 53;


// --------- Sent packets ------ //

struct Motor {
  uint32_t seq;
  float left_motor;
  float right_motor;
};

Motor motor;
int motor_id = 32;
uint32_t motor_timer = 0;

struct Led{
  char command[10];
};


Led led;
int led_id = 87;
uint32_t led_timer = 0;



void updateSystem(){

    if (ros::Time::now().toNSec()/1e6 - last_time > 50){

        //motor.seq++;
        last_time = ros::Time::now().toNSec()/1e6;
    }

}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "serial_jitbus_test");
	ros::NodeHandle n;

	jit.init("/dev/ttyUSB0", 9600);
 
	ros::Rate loop_rate(5000);

    encoder.seq = 0;
    encoder.left_encoder = 0;
    encoder.right_encoder = 0;

    sensor.seq = 0;
    sensor.left_sensor = 0;
    sensor.right_sensor = 0;

    motor.seq = 8;
    motor.left_motor = 3.14;
    motor.right_motor = 9.86;

    memcpy(led.command, "blink,10,1", sizeof(led.command));


	while (ros::ok()){

        jit.sendPacketHz(motor, motor_id, motor_timer, 100);
        jit.sendPacketHz(led, led_id, led_timer, 100);

        if (jit.available() > 0){
           
            if (jit.receivePacket(encoder, encoder_id)){

                ROS_INFO("encoder seq %d", encoder.seq);
            }

            if (jit.receivePacket(sensor, sensor_id)){

                ROS_INFO("sensor seq %d", sensor.seq);
            }

        }

        updateSystem();

        if (ros::Time::now().toNSec()/1e6 - last_time_debug > 1000){

            //ROS_INFO("pump");
            last_time_debug = ros::Time::now().toNSec()/1e6;
        }
	
		ros::spinOnce();
		loop_rate.sleep();
	}

}