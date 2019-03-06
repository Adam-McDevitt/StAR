'''
- script used to test individual motors
- used for debugging
- author ilie
'''


import numpy as np
import smbus
import time
import requests
import time
from multiprocessing import Process, Manager, Lock

bus = smbus.SMBus(1)
address = 0x04
movement = ''
#---------------------------------------------------------------------------
#                              Useful functions
#---------------------------------------------------------------------------

# Writing to motorboard
def motor_move(id, power):
    """
    Smooth control of the motors
    Mode 2 is Forward.
    Mode 3 is Backwards.
    """
    mode = 2 if power >= 0 else 3
    cmd_byte = id << 5 | 24 | mode << 1
    pwr = int(abs(power) * 2.55)
    bus.write_i2c_block_data(address, 0, [cmd_byte, pwr])

def write(value):
    bus.write_byte_data(address, 0x00, value)

def stopMotor(id):
    """
    Mode 0 floats the motor.
    """
    direction = 0
    byte1 = id << 5 | 16 | direction << 1
    write(byte1)


def stopMotors():
    """
    The motor board stops all motors if bit 0 is high.
    """
    write(0x01)


#---------------------------------------------------------------------------
#                     Command to control motor
#---------------------------------------------------------------------------
# positive ex 50 - down
# negative ex -50 - up
# motor_move(4,  -90)
# motor_move(2, 90)
# time.sleep(4)
# motor_move(1, -100)
# motor_move(3, -100)
print('ON')
# motor_move(5,100)
# motor_move(1, -75)
# motor_move(2, 75)
#motor_move(5, 100)
#time.sleep(3)



#stopMotors()
#print('OFF')



'''
Motor movement directions
`           + | -`
motor 1:    ->|<-
motor 2:    up|down
motor 3:    ->|<-
motor 4:    up|down
'''
# Move down slow
#motor_move(5,100)
motor_move(1, 100)
#motor_move(2, 100)
motor_move(3, 100)
#motor_move(4, -100)
time.sleep(2)
stopMotors()
# time.sleep(2)
# motor_move(1, 40)
# time.sleep(4)
# stopMotors()
# time.sleep(2)
# motor_move(1, 40)
# time.sleep(4)
# stopMotors()
