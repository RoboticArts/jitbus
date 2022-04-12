#!/usr/bin/env python

import time
import ctypes
import rospy
from jitbus.serial_jitbus import SerialJitbus

jit = SerialJitbus()
last_time = 0
last_time_debug = 0

# --------- Received packets ------ #

class Encoder(ctypes.Structure):
    _fields_ = [('seq', ctypes.c_uint32),
                ('left_encoder', ctypes.c_uint32),
                ('right_encoder', ctypes.c_uint32 )]

encoder = Encoder()
encoder_id = 77

class Sensor(ctypes.Structure):
    _fields_ = [('seq', ctypes.c_uint32),
                ('left_sensor', ctypes.c_uint16),
                ('right_sensor', ctypes.c_uint16 )]

sensor = Sensor()
sensor_id = 53


# --------- Sent packets --------- #

class Motor(ctypes.Structure):
    _fields_ = [('seq', ctypes.c_uint32),
                ('left_motor', ctypes.c_float),
                ('right_motor', ctypes.c_float )]

motor = Motor()
motor_id = 32
motor_timer = [0]

class Led(ctypes.Structure):
    _fields_ = [('command', ctypes.c_char*10)]

led = Led()
led_id = 87
led_timer = [0]
led.command = "TURN_ON"

def updateSystem():

    global last_time

    if jit.time_ms() - last_time > 50:

        motor.seq = motor.seq + 1
        last_time = jit.time_ms()

        if motor.seq%10 == 0 and led.command == "TURN_ON":
            led.command = "TURN_OFF"
        elif motor.seq%10 == 0 and led.command == "TURN_OFF":
            led.command = "TURN_ON"





if __name__ == "__main__":
    
    try:

        jit.init('/dev/ttyUSB0', 9600)

        encoder.seq = 0
        encoder.left_encoder = 1
        encoder.right_encoder = 2

        sensor.seq = 0
        sensor.left_sensor = 3
        sensor.right_sensor = 4

        jit.print_set_level(3)

        rospy.init_node('serial_jitbus_test', anonymous=False)
        rate = rospy.Rate(5000)

        while not rospy.is_shutdown():

            jit.sendPacketHz(motor, motor_id, motor_timer, 100)
            jit.sendPacketHz(led, led_id, led_timer, 100)

            if jit.available():

                encoder_success, encoder_data  = jit.receivePacket(Encoder, encoder_id)
                    
                if encoder_success:
                    print("encoder seq " + str(encoder_data.seq))

                sensor_success, sensor_data = jit.receivePacket(Sensor, sensor_id)

                if sensor_success:
                    print("sensor seq " + str(sensor_data.seq))

            updateSystem()

            if (jit.time_ms() - last_time_debug > 1000):

                # print("pump")
                last_time_debug = jit.time_ms()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass