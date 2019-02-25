import numpy as np
import smbus
import time
import requests
import time
# import server
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

# Gradual Movement function
def gradual_start(dir, time_move):
    print('Starting motors')
    time_inc = 0.2
    if dir == 'FORWARD':
        print('FORWARD')
        for i in range(1,6):
            print(i,i*20)
            motor_move(2,-20*i)
            motor_move(4, 20*i)
            time.sleep(time_inc)
            if i == 5:
                print('Got to top speed')
                keep_moving(dir, (time_move-(5*time_inc)))
    elif(dir == 'BACKWARD'):
        print('BACKWARD')
        for i in range(1,6):
            motor_move(2, 20*i)
            motor_move(4,-20*i)
            time.sleep(time_inc)
            print(i,i*20)
            if i == 5:
                print('Got to top speed')
                keep_moving(dir, (time_move-5*time_inc))
    elif(dir == 'RIGHT'):
        print('RIGHT')
        for i in range(1,6):
            motor_move(1,20*i)
            motor_move(3,20*i)
            time.sleep(time_inc)
            if i == 5:
                print('Got to top speed')
                keep_moving(dir, (time_move-5*time_inc))
    elif(dir == 'LEFT'):
        print('LEFT')
        for i in range(1,6):
            motor_move(1,-20*i)
            motor_move(3,-20*i)
            time.sleep(time_inc)
            if i == 5:
                print('Got to top speed')
                keep_moving(dir, (time_move-5*time_inc))

# Keep moving
def keep_moving(dir, time_left):
    keep_moving_delay = 0.1
    print('time_left=',time_left)
    if dir == 'FORWARD':
        print('Moving FORWARD')
        motor_move(2,-100)
        motor_move(4, 100)
        time.sleep(time_left)
        stopMotors()
        time.sleep(keep_moving_delay)
    elif dir == 'BACKWARD':
        print('Moving BACKWARD')
        motor_move(2, 100)
        motor_move(4,-100)
        time.sleep(time_left)
        stopMotors()
        print('Shuting down motors')
        time.sleep(keep_moving_delay)
    elif dir == 'RIGHT':
        print('Moving RIGHT')
        motor_move(1,100)
        motor_move(3,100)
        time.sleep(time_left)
        stopMotors()
        time.sleep(keep_moving_delay)
    elif dir == 'LEFT':
        print('Moving LEFT')
        motor_move(1,-100)
        motor_move(3,-100)
        time.sleep(time_left)
        stopMotors()
        time.sleep(keep_moving_delay)
    else:
        print('COMMAND NOT RECOGNIZED')
    print('Done moving')
    time.sleep(keep_moving_delay)

def quick_rotation(direction, time_move):
    if(direction == 'RIGHT'):
        print('Right rotation')
        motor_move(1,-100)
        motor_move(2,-100)
        motor_move(3, 100)
        motor_move(4,-100)
        time.sleep(time_move)
        stopMotors()
    elif(direction == 'LEFT'):
        print('LEFT rotation')
        motor_move(1, 100)
        motor_move(2, 100)
        motor_move(3,-100)
        motor_move(4, 100)
        time.sleep(time_move)
        stopMotors()
    else:
        print('COMMAND NOT RECOGNIZED')
    print('Done quick rotation')

def move_diagonal(dir, time_move):
    if dir == 'RIGHTUP':
        motor_move(2,-100)
        motor_move(4, 100)
        motor_move(1, 100)
        motor_move(3, 100)
        time.sleep(time_move)
        stopMotors()
    elif dir == 'RIGHTDOWN':
        motor_move(2, 100)
        motor_move(4,-100)
        motor_move(1, 100)
        motor_move(3, 100)
        time.sleep(time_move)
        stopMotors()
    elif dir == 'LEFTUP':
        motor_move(2,-100)
        motor_move(4, 100)
        motor_move(1,-100)
        motor_move(3,-100)
        time.sleep(time_move)
        stopMotors()
    elif dir == 'LEFTDOWN':
        motor_move(2, 100)
        motor_move(4,-100)
        motor_move(1,-100)
        motor_move(3,-100)
        time.sleep(time_move)
        stopMotors()
    else:
        print('COMMAND NOT RECOGNIZED')
        stopMotors()
#-----------------------------------------------------------------------------


#-----------------------------------------------------------------------
#                       Writing Commands
#-----------------------------------------------------------------------
'''
gradual_start('FORWARD', 3)
time.sleep(2)
gradual_start('LEFT', 4)
time.sleep(2)
gradual_start('BACKWARD',5)
time.sleep(2)
gradual_start('RIGHT',4)
time.sleep(2)
quick_rotation('RIGHT',4)
time.sleep(2)
quick_rotation('LEFT',4)
time.sleep(1)
# stopMotors()
'''
#-------------------------------------------------------------------------------
#                   Movement testing
#-------------------------------------------------------------------------------


# quick_rotation('LEFT', 4)

#-------------------------------------------------------------------------------
#                   Look around
#-------------------------------------------------------------------------------
'''
quick_rotation('RIGHT',0.4)
time.sleep(0.5)
quick_rotation('RIGHT',0.3)
time.sleep(0.5)
quick_rotation('LEFT',0.3)
time.sleep(0.5)
quick_rotation('RIGHT',0.3)
time.sleep(0.5)
quick_rotation('LEFT',0.3)
time.sleep(0.5)
quick_rotation('LEFT',0.55)
time.sleep(0.5)
quick_rotation('LEFT',0.3)
time.sleep(0.5)
quick_rotation('RIGHT',0.3)
time.sleep(0.5)
quick_rotation('LEFT',0.3)
time.sleep(0.5)
quick_rotation('RIGHT',0.45)
time.sleep(0.5)
gradual_start('FORWARD',3)
time.sleep(1)
quick_rotation('LEFT',0.65)
time.sleep(1)
quick_rotation('LEFT',1.15)
'''

#-------------------------------------------------------------------------------
#                   Spiral
#-------------------------------------------------------------------------------
'''
gradual_start('FORWARD',2)
time.sleep(0.5)
gradual_start('LEFT',2)
time.sleep(0.5)
gradual_start('BACKWARD',2)
time.sleep(0.5)
gradual_start('RIGHT',2)
time.sleep(0.5)


#-------------------------------------------------------------------------------
#                            Diagonal
#-------------------------------------------------------------------------------
move_diagonal('RIGHTUP',1)
time.sleep(0.5)
move_diagonal('RIGHTDOWN',1)
time.sleep(0.5)
move_diagonal('LEFTDOWN',1)
time.sleep(0.5)
move_diagonal('LEFTUP',1)
'''
# motor_move(2,-100)
# motor_move(4,100)
quick_rotation('RIGHT',5)
time.sleep(5)
stopMotors()
