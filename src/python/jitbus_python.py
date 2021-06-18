#!/usr/bin/python2.7

# from jitbus._jitbus_binding_so import JitbusBinding

# myJitbus = JitbusBinding()

# msg = myJitbus.say("hello world!")

# myJitbus.sendPacket("aaa", 4)

# print(msg)


# UINT8_T = 'B'
# UINT16_T = 'H'
# UINT32_T = 'L'
# UINT64_T = 'Q'

# INT8_T = 'b'
# INT16_T = 'h'
# INT32_T = 'l'
# INT64_T = 'q'

# FLOAT = 'f'
# DOUBLE = 'd'


import struct

# packed = struct.pack('i 4s f', 10, b'John', 2500)
# print(repr(packed))

# unpacked = struct.unpack('i 4s f', packed)
# print(unpacked)

# class pyStruct(object):

#     def __init__(self):
 
#         object.__setattr__(self, "_types", {})
#         object.__setattr__(self, "_sizes", {})

#     def set_type(self, name, _type, _size):
#         self._types[name] = _type
#         self._sizes[name] = _size

#     def __setattr__(self, name, value):
        
#         _type = self._types[name]
#         _sizes = self._sizes[name]
        
#         value = struct.pack(_type, value)
       
#         object.__setattr__(self, name, value)


# myStruct = pyStruct()
# myStruct.set_type('x', UINT8_T, 1)
# myStruct.set_type('y', UINT16_T, 2)
# myStruct.set_type('z', UINT8_T, 1)
# myStruct.x = 1
# myStruct.y = 32000
# myStruct.z = 5


# print(repr(myStruct.x))
# print(repr(myStruct.y))
# print(repr(myStruct.z))


from ctypes import * 

class Imu(Structure):
    _fields_ = [("x", c_uint32*2),
                ("y", c_uint32)]

imu = Imu()
imu.x[0] = 2
imu.x[1] = 3
imu.y = 5

print(imu.x[0])
print(imu.x[1])
print(imu.y)


# class pyStruct(Structure):
    
#     def set(self, Structure, _name, _type, _elements):
#         _fields_.append((_name, _type * _elements))

# myStruct = pyStruct()
# myStruct.set('x', c_int, 2)

#myStruct.a = struct.pack(UINT8_T, 3)
#print(repr(myStruct.a))

#pack_1 = struct.pack(UINT8_T, 3)
#print(repr(pack_1))
