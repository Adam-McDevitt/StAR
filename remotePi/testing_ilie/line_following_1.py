'''
- script used for line following
- uses an fsm to function
- used for debugging
- author: ilie, pieris

Motor movement directions
`           + | -`
motor 1:    <-|->
motor 2:    up|down
motor 3:    ->|<-
motor 4:  down|up
'''

import numpy as np
import smbus
import time
import requests
import time
import serial
import re
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
ERROR_calib = 3500  # calibration error - perfectly centered

motors_speed = 0      # global used for speed of right mototrs
line_last_error    = 0      # global that holds last error from fwd sensors
line_new_error     = 0
mottor_adjustment  = 0       # global used to readjust mottor speed

right_motors_default_speed = 50    # global used as default speed of right motors
left_motors_default_speed  = 50    # global used as default speed of left motors
right_max_speed = 100              # max speed of right mototrs
left_max_speed  = 100              # max speed of left motors

range_left=2500
range_right=5500

motor1_old=0
motor2_old=0
motor3_old=0
motor4_old=0

motor1_current=0
motor2_current=0
motor3_current=0
motor4_current=0


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
#                           Movement functions
#-------------------------------------------------------------------------------
def turn_right(speed_input):
    global motor1_old,motor2_old,motor3_old,motor4_old
    
    if(abs(60 + speed_input)>100):
        speed_input_horizontal = 100
    else:
        speed_input_horizontal = speed_input + 60     

    motor1_current=speed_input_horizontal
    motor3_current=speed_input_horizontal
    
    if(abs(50 + speed_input > 80)):
        speed_input_forward = 80
    else:
        speed_input_forward = 50 + speed_input
    if (check_change_rotation()==True):

        #print('ACTUALLY TURNING RIGHT WITH SPEED:',speed_input_horizontal)
        
        motor_move(1, -1*speed_input_horizontal)
        motor_move(2, speed_input_forward)
        motor_move(3, -1*speed_input_horizontal)
        motor_move(4, -1*speed_input_forward)

        motor1_old = -1*speed_input_horizontal
        motor2_old = speed_input_forward
        motor3_old = -1*speed_input_horizontal
        motor4_old = -1*speed_input_forward

def turn_left(speed_input):
    global motor1_old,motor2_old,motor3_old,motor4_old
    
    if(abs(60 + speed_input)>100):
        speed_input_horizontal = 100
    else:
        speed_input_horizontal = speed_input + 60     
    
    motor1_current=speed_input_horizontal
    motor3_current=speed_input_horizontal

    if(abs(50 + speed_input > 80)):
        speed_input_forward = 80
    else:
        speed_input_forward = 50 + speed_input
    if (check_change_rotation()==True):
        #print('ACTUALLY TURNING LEFT WITH SPEED:',speed_input_horizontal)

        motor_move(1, speed_input_horizontal)
        motor_move(2, speed_input_forward)
        motor_move(3, speed_input_horizontal)
        motor_move(4, -1*speed_input_forward)
        

        motor1_old=speed_input_horizontal
        motor2_old=speed_input_forward
        motor3_old=speed_input_horizontal
        motor4_old=-1*speed_input_forward

def check_change_forward():
    global motor2_current,motor2_old,motor4_current,motor4_old
    if((abs(motor2_old-motor2_current)>8) or (abs(motor4_old-motor4_current)>8)):
        motor2_old=motor2_current
        motor4_old=motor4_current
        return True
    else:
        return False

def check_change_rotation():
    global motor1_current,motor1_old,motor3_current,motor3_old
    if(abs(motor1_old-motor1_current)>8):
        motor1_old=motor1_current
        motor3_old=motor3_current
        return True    
    else:
        return False    
#-------------------------------------------------------------------------------
#                           Main Loop
#-------------------------------------------------------------------------------

if __name__== "__main__":
    
    # Globals
    #global ser
    #global line_new_error
    # Main FSM
    ser = serial.Serial('/dev/ttyACM0',9600)        # connect to arduino
    #print('serial name: ',ser)
    # start process
    start_process = True
    # sensor_fwd_reading = 3100
    while start_process:
        #print('INSIDE WHILE')
            #------------DEBUGGING--------------#
        # print(counter)
        # if (counter >= 2000):# and counter < 5000):
        #     sensor_fwd_reading -=20
            # time.sleep(0.5)
        # if (counter >= 5000):
        #     sensor_fwd_reading += 100
        # if (counter == 8000):
        #     counter = 0
        counter+=1
            #-----------------------------------#

        try:
            #----------READ SERIAL------------#
            # read serial
            ser_line=ser.readline()                 # read newest line from serial
            sensor_values=ser_line.split('\t')      # all values from the senors
            #print('sensor_values = ', sensor_values)
            sensor_fwd_reading_string = sensor_values[-1]  # position of the sensors
            sensor_fwd_reading= int(re.sub('[^0-9]','', sensor_fwd_reading_string))
            #print('sensor_fwd_reading = ', sensor_fwd_reading)
            #-----------------------------------#

            #-------CALCUALTE MOTOR SPEED-------#
            motors_speed_old = motors_speed
            line_new_error = sensor_fwd_reading - ERROR_calib
            mottor_adjustment = Kp * line_new_error + Kd * (line_new_error - line_last_error)
            line_last_error = line_new_error
            motors_speed = mottor_adjustment

            if(motors_speed > 100):
                motors_speed = 100
            elif(motors_speed < -100):
                motors_speed = -100
            motors_speed=abs(motors_speed)
            #print('motors_speed =', motors_speed)
            #-----------------------------------#
        except Exception, e:
            pass



            #-----------MOVE MOTORS-------------#
        if (sensor_fwd_reading < range_left):
            #print('TURNING LEFT WITH SPEED:',motors_speed*3)
            motor1_current=motors_speed
            motor3_current=motors_speed
            try:
                turn_left(motors_speed)
            except Exception, e:
                pass
        elif(sensor_fwd_reading > range_right):
            #print('TURNING RIGHT')
            try:
                turn_right(motors_speed)
            except Exception, e:
                pass
        elif(sensor_fwd_reading>=range_left and sensor_fwd_reading<=range_right):
            #print('MOVING FORWARD')
            motor2_current=70
            motor4_current=-70
            try:
                if(check_change_forward()==True):
                    stopMotor(1)
                    stopMotor(3)
                    motor_move(2,70)
                    motor_move(4,-70)
                    #print('INSIDE FORWARD')
                    motor2_old=motor2_current
                    motor4_old=motor4_current
                    pass
                else:
                    pass
            except Exception, e:
                pass            
            #-----------------------------------#


            #------------DEBUGGING--------------#

        # print('Kd = {} Kp = {}'.format(Kp,Kd))

        # motor_move(2, 70)
        # motor_move(4,-70)
        # print('motors_speed', int(motors_speed))
        # motor_move(1,-motors_speed)
        # motor_move(3,motors_speed)























##
