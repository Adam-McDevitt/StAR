'''
- script used for line following
- uses an fsm to function
- used for debugging
- author: ilie
'''

import numpy as np
import smbus
import time
import requests
import time
import serial
from multiprocessing import Process, Manager, Lock

#-------------------------------------------------------------------------------
#                              Globals
#-------------------------------------------------------------------------------

bus = smbus.SMBus(1)        # connection from Pi to motor board
address = 0x04              # address from Pi to motor board
movement = ''               # global used to distiguish movement
ser = ''                    # global used to read & write to serial
sensor_fwd_reading = 0      # global new reading from forward sensors
sensor_fwd_reading_old = [] # global old readings from forward sensors

Kp = 0.01              # proportional error
# Ki =              # integral     error
Kd = 0.05             # derivative   error
ERROR_calib = 3100  # calibration error - perfectly centered

motors_speed = 0      # global used for speed of right mototrs
line_last_error    = 0      # global that holds last error from fwd sensors

mottor_adjustment = 0       # global used to readjust mottor speed

right_motors_default_speed = 50    # global used as default speed of right motors
left_motors_default_speed  = 50    # global used as default speed of left motors
right_max_speed = 100              # max speed of right mototrs
left_max_speed  = 100              # max speed of left motors


counter = 0
#-------------------------------------------------------------------------------
#                              Useful functions
#-------------------------------------------------------------------------------

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



#-------------------------------------------------------------------------------
#                           Main Loop
#-------------------------------------------------------------------------------

if __name__== "__main__":

    # Globals
    global ser

    # Main FSM
    # ser = serial.Serial('/dev/ttyACM0',9600)        # connect to arduino
    print('serial name: ',ser)
    # start process
    start_process = True
    sensor_fwd_reading = 3100
    while start_process:
        print(counter)
        if (counter == 1000):
            sensor_fwd_reading +=100
        counter+=1

        # read serial
        # ser_line=ser.readline()                 # read newest line from serial
        # sensor_values=line.split('\t')          # all values from the senors
        # sensor_fwd_reading = sensor_values[-1]  # position of the sensors

        line_new_error = sensor_fwd_reading - ERROR_calib

        mottor_adjustment = Kp * line_new_error + Kd * (line_new_error - line_last_error)
        line_last_error = line_new_error

        motors_speed = mottor_adjustment

        ################
        # PRINTING
        ###############
        print('Kd = {} Kp = {}'.format(Kp,Kd))





        if(motors_speed > 100):
            motors_speed = 100
        elif(motors_speed < -100):
            motors_speed = -100

        motor_move(2, 60)
        motor_move(4,-60)

        print('motors_speed', motors_speed)


        motor_move(1,motors_speed)
        motor_move(3,motors_speed)

























##