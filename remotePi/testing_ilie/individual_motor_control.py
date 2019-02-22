'''
Script used to control individual motor movement of the robot
Version: 1.0
Usage: Debugging and correction of robot direction
Comments: Need to add rotation function and control of side motors
          Need more functions involving moevemt for a prticular amount of time
          Use: http://veemon:5000/sendinstructions?inst=X to sedn commands
          X is the variale/command you pass
Contributions: Ilie, Adam

            command     |   actions
            ------------+----------
               0        |   stop all
               1        |   start fwd
               2        |   inc mot 2
               3        |   inc mot 4
               4        |   dec mot 2
               5        |   dec mot 4
               6        |   idle state
'''
import numpy as np
import smbus
import time
import server
import requests
import time
from multiprocessing import Process, Manager, Lock
#-------------------------------------------------------------------------------
#                           Global Variables
#-------------------------------------------------------------------------------
trig_input_changed = 0
input_command      = 0
input_command_old  = [0,0,0]

time_wait          = 0.1
delta_power        = 10

motor_2_pow        = 0
motor_4_pow        = 0

bus = smbus.SMBus(1)
address = 0x04
#-------------------------------------------------------------------------------
#                           Motor Communication
#-------------------------------------------------------------------------------

# Writing to motorboard
def motor_move(id, power):
    """
    Smooth control of the motors
    Mode 2 is Forward.
    Mode 3 is Backwards.
    """
    global bus
    global address

    mode = 2 if power >= 0 else 3
    cmd_byte = id << 5 | 24 | mode << 1
    pwr = int(abs(power) * 2.55)
    bus.write_i2c_block_data(address, 0, [cmd_byte, pwr])

def write(value):
    global bus
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
    The motor boMotor Communicationard stops all motors if bit 0 is high.
    """
    write(0x01)


#---------------------------------------------------------------------------
#                           Movement functions
#---------------------------------------------------------------------------

def start_move_fwd():
    global motor_2_pow
    global motor_4_pow

    motor_2_pow = -20
    motor_4_pow =  20
    motor_move(2, motor_2_pow)
    motor_move(4,  motor_4_pow)

def inc_fwd_pow(command):
    global motor_2_pow
    global motor_4_pow
    global delta_power

    if(command==2):
        print('Increasing mototor 2 power by',delta_power)
        motor_2_pow = motor_2_pow - delta_power
        motor_move(2, motor_2_pow)
    elif(command==3):
        print('Increasing mototor 4 power by',delta_power)
        motor_4_pow = motor_4_pow + delta_power
        motor_move(4, motor_4_pow)
    else:
        print('inc_fwd_pow() command not recognized')

def dec_fwd_pow(command):
    if(command==4):
        print('Decreasing mototor 2 power by',delta_power)
        motor_2_pow = motor_2_pow + delta_power
        motor_move(2, motor_2_pow)
    elif(command==5):
        print('Decreasing mototor 4 power by',delta_power)
        motor_4_pow = motor_4_pow - delta_power
        motor_move(4, motor_4_pow)
    else:
        print('dec_fwd_pow if(command==__):() command not recognized')


def inc_fwd_pow_both(command):
    global motor_2_pow
    global motor_4_pow
    global delta_power

    if(command==7):
        print('Increasing both motor 2 and 4 by ',delta_power)
        inc_fwd_pow(3)
        inc_fwd_pow(2)
    else:
        print('inc_fwd_pow() command not recognized')

def dec_fwd_pow_both(command):
    global motor_2_pow
    global motor_4_pow
    global delta_power

    if(command==8):
        print('Decreasing both motor 2 and 4 by ',delta_power)
        dec_fwd_pow(4)
        dec_fwd_pow(5)
    else:
        print('inc_fwd_pow() command not recognized')
#-------------------------------------------------------------------------------
#                           FSM Functions
#-------------------------------------------------------------------------------

def perform_command(command):
    global trig_input_changed

    trig_input_changed = 0
    if(command==0):
        stopMotors()
    elif(command==1):
        start_move_fwd()
    elif(command==2):
        inc_fwd_pow(command)
    elif(command==3):
        inc_fwd_pow(command)
    elif(command==4):
        dec_fwd_pow(command)
    elif(command==5):
        dec_fwd_pow(command)
    elif(command==7):
        inc_fwd_pow_both(command)
    elif(command==8):
        dec_fwd_pow_both(command)
    elif(command==6):
        pass
    else:
        print('perform_command() command not recognized')

def check_if_input_changed(command):
    global input_command
    global input_command_old
    global trig_input_changed
    try:
        if(command==input_command_old[-1]):
            trig_input_changed = 0
            print('input hasn\'t changed')
        else:
            print('input has changed')
            trig_input_changed = 1
            input_command_old.append(input_command)
            input_command = command
    except Exception as e:
        print(e)
        print('Exception occured in check_if_input_changed()')
        time.sleep(2)

def wait_till_checking_again():
    global time_wait
    time.sleep(time_wait)

#-------------------------------------------------------------------------------
#                           Main Loop
#-------------------------------------------------------------------------------

if __name__== "__main__":

    a=9
    b=1

    process = Process(target=server.run)
    process.start()

    while True:
        pass
        # print(val)
        try:
            print('contacting server')
            r = requests.get('http://0.0.0.0:5000/getinstructions')

            print('\nr = {}\n'.format(int(r.text)))

            check_if_input_changed(int(r.text))

            if(trig_input_changed==1):
                print('trig_input_changed==1')
                perform_command(int(r.text))
            elif(trig_input_changed==0):
                print('trig_input_changed==0')
                wait_till_checking_again()
            else:
                print('There is and issue with trig_input_changed')


        except Exception as e:
            print(e)
            print()
            print("will retry in 5")
            time.sleep(5)
