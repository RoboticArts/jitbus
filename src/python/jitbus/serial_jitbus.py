#!/usr/bin/python2.7
import sys
import time
import serial
import struct
from jitbus_python import Jitcore

JITBUS_BUFFER_SIZE=1000

class SerialJitbus(Jitcore):

    def __init__(self):
        
        Jitcore.__init__(self) 
        self.start_time = round(time.time() * 1000)
        self.CONNECTED = 0
        self.DISCONNECTED = 1
        
        self.cbuffer = []
        self.last_time = 0
        self.last_state = self.CONNECTED
        self.current_state = self.DISCONNECTED 

        self.serial_port = serial.Serial()     

    def __del__(self):
        
        if self.current_state == self.CONNECTED:
            self.disconnect()

    def init(self, port, baudrate, timeout = 50, reconnection_time = 2):
        
        self.print_enable_color(True)
        self.print_set_level(self.INFO)
        self.reconnection_time = reconnection_time

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.connect()

    def connect(self):

        succes = False

        try:

            self.serial_port.port = self.port
            self.serial_port.baudrate = self.baudrate
            self.serial_port.timeout = self.timeout
            self.serial_port.bytesize = serial.EIGHTBITS
            self.serial_port.parity = serial.PARITY_NONE
            self.serial_port.stopbits = serial.STOPBITS_ONE

            self.serial_port.open()

            self.print_info("jitbus::connect -> Serial port " + str(self.port) + " opened")
            
            self.switchState(self.CONNECTED)
            success = True

        except serial.SerialException as e:
            self.print_error("jitbus::connect -> " + str(e))
            self.disconnect()
            success = False
    
        except ValueError as e:
            self.print_error("jitbus::connect -> " + str(e))
            self.disconnect()
            success = False

        return success

    def disconnect(self):

        self.serial_port.close()
        self.switchState(self.DISCONNECTED)
    
    def connected(self):

        is_connected = False

        if (self.current_state == self.CONNECTED):

            is_connected = True
            self.last_time = self.time_ms()/1000
        
        else:
            
            current_time = self.time_ms()/1000

            if current_time - self.last_time >= self.reconnection_time:

                is_connected = self.connect()
                self.last_time = self.time_ms()/1000

        return is_connected

    def switchState(self, state):

        if state != self.current_state:
            self.last_state = self.current_state
            self.current_state = state

    def read(self):

        return self.cbuffer.pop(0)
    
    def write(self, data):
  
        success = False
        
        if self.connected():

            try:

                data_byte = bytearray()
                data_byte.append(data)

                bytes_written = self.serial_port.write(data_byte)

                if bytes_written <= 0:
                    success = False
                    self.print_warn("jitbus::write -> Byte hex: " + str(data) + " not sent, I will try again")
                else:
                    success = True

            except serial.SerialException as e:
                self.print_error("jitbus::connect -> " + str(e))
                self.disconnect()
                success = False

            except ValueError as e:
                self.print_error("jitbus::connect -> " + str(e))
                self.disconnect()
                success = False

        return success

    def available_read(self):

        if self.connected():

            if (self.serial_port.inWaiting() >= 63): # 64 bytes buffer

                self.print_warn("SerialJitbus::available_bytes -> Serial RX buffer is full, "
                                "execution frequency is too low or sender is too fast. Next "
                                "bytes could be lost")
            
            if len(self.cbuffer) >= JITBUS_BUFFER_SIZE:

                self.print_warn("SerialJitbus::available_bytes -> Serial RX circular buffer is "
                                "full, execution frequency is too low or sender is too fast. Next "
                                "bytes could be lost")
                
            while self.serial_port.inWaiting() > 0:

                if self.connected():

                    try:
                        data_byte = ord(self.serial_port.read())
                        self.cbuffer.append(data_byte)

                    except serial.SerialException as e:
                        self.print_error("jitbus::connect -> " + str(e))
                        self.disconnect()
                        success = False

                    except ValueError as e:
                        self.print_error("jitbus::connect -> " + str(e))
                        self.disconnect()
                        success = False

        available_data = len(self.cbuffer)

        return available_data
    
    def available_write(self):

        virutal_serial_size = 64

        return virutal_serial_size
    
    def time_ms(self):
        current_time = round(time.time() * 1000) 
        elapsed_time_since_start = int(current_time - self.start_time)
        return elapsed_time_since_start

    def print_log(self, message):
        sys.stdout.write(message)

