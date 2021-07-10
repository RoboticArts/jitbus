#define JITBUS_DISABLE_LOG
#include <jitbus.h>

SerialJitbus jit;
uint32_t last_time = 0;
uint32_t last_time_debug = 0;

// --------- Sent packets ------ //

struct Encoder {
  uint32_t seq;
  uint32_t left_encoder;
  uint32_t right_encoder;

};

Encoder encoder;
int encoder_id = 77;
uint32_t encoder_timer = 0;

struct Sensor {
  uint32_t seq;
  uint16_t left_sensor;
  uint16_t right_sensor;
};

Sensor sensor;
int sensor_id = 53;
uint32_t sensor_timer = 0;


// --------- Received packets ------ //

struct Motor {
  uint32_t seq;
  float left_motor;
  float right_motor;
};

Motor motor;
int motor_id = 32;

struct Led{
  char command[10];
};


Led led;
int led_id = 87;



void setup() {

Serial.begin(9600);
Serial1.begin(9600);
jit.begin(Serial1, Serial);
jit.waitSerialUSB();

encoder.seq = 0;
encoder.left_encoder = 3;
encoder.right_encoder = 7;

sensor.seq = 0;
sensor.left_sensor = 4;
sensor.right_sensor = 9;

motor.seq = 0;
motor.left_motor = 0.0;
motor.right_motor = 0.0;

memcpy(led.command, "abcdefghij", sizeof(led.command));

}


void updateSystem(){

  if(millis()-last_time > 50){

    encoder.seq++;
    sensor.seq++;
    last_time = millis();
   }

}

void loop() {


  jit.sendPacketHz(encoder, encoder_id, encoder_timer, 100);
  jit.sendPacketHz(sensor, sensor_id, sensor_timer, 100);

  if (jit.available() > 0){

      if (jit.receivePacket(motor, motor_id)){

          jit.print_info("motor seq %d", motor.seq);
      }

      if (jit.receivePacket(led, led_id)){

          jit.print_info("sensor seq %d", sensor.seq);
      }

  }


  if(millis()-last_time_debug > 1000){

    jit.print_info("pump");
    last_time_debug = millis();
   }

  updateSystem();
  

}
