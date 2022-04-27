#!/usr/bin/python2.7
from jitbus._jitbus_binding_so import JitbusBinding
from ctypes import* 

class Jitcore(JitbusBinding):
    
    def __init__(self):
        JitbusBinding.__init__(self) 
        self.DEBUG = 4
        self.INFO = 3
        self.WARN = 2
        self.ERROR = 1
        self.FATAL_ERROR = 0

    def available(self):
        return self.available_py()

    def sendPacket(self, data, data_id):
        return self.sendPacket_py(self.pack(data), sizeof(data), data_id)

    def receivePacket(self, data_type, data_id):
        aux = 'n' * sizeof(data_type)
        success = self.receivePacket_py(aux, data_id)
        data = self.unpack(data_type, aux)
        return success, data

    def sendPacketBlocking(self, data, data_id, timeout = 0):
        return self.sendPacketBlocking_py(self.pack(data), sizeof(data), data_id, timeout)

    def sendPacketHz(self, data, data_id, time_list, frequency):
        time_list[0], is_sent = self.sendPacketHz_py(self.pack(data), sizeof(data), data_id, time_list[0], frequency)
        return is_sent

    def pack(self, ctype_instance):
        buf = string_at(byref(ctype_instance), sizeof(ctype_instance))
        return buf

    def unpack(self, ctype, buf):
        cstring = create_string_buffer(buf)
        ctype_instance = cast(pointer(cstring), POINTER(ctype)).contents
        return ctype_instance

    def print_debug(self, message):
        self.print_debug_py(message)

    def print_info(self, message):
        self.print_info_py(message)

    def print_warn(self, message):
        self.print_warn_py(message)

    def print_error(self, message):
        self.print_error_py(message)

    def print_fatal_error(self, message):
        self.print_fatal_error_py(message)

    def print_set_level(self, level):
        self.print_set_level_py(level)

    def print_enable_color(self, enabled):
        self.print_enable_color_py(enabled)

# https://hyunyoung2.github.io/2018/05/11/How_To_Use_Ctypes_In_Python2/
# https://stackoverflow.com/questions/1825715/how-to-pack-and-unpack-using-ctypes-structure-str


#msg = repr(''.join(buffer(imu)))
#print(msg)


# myJitbus = JitPython()
# myJitbus.sendPacket(imu, 4)
# myJitbus.read2()


# data_id = 5
# success, data = receivePacket(Imu, data_id)

# if success:
#     print(data.x[0])
#     print(data.x[1])
#     print(data.y)
# else:
#     print("Receive Error")



