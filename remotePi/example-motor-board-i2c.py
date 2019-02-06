import numpy as np
import smbus
import time

bus = smbus.SMBus(1)
address = 0x04


def write(value):
    bus.write_byte_data(address, 0x00, value)

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


def setMotor(id, speed):
    """
    Mode 2 is Forward.
    Mode 3 is Backwards.
    """
    direction = 2 if speed >= 0 else 3
    speed = np.clip(abs(speed), 0, 100)
    byte1 = id << 5 | 24 | direction << 1
    byte2 = int(speed * 2.55)
    write(byte1)
    write(byte2)


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

def stopwatch_run(miliseconds):
    start = time.time()*1000 # in miliseconds
    time.clock()    
    elapsed = 0
    while elapsed < miliseconds:
      elapsed = time.time()*1000 - start
      setMotor(4,70)

def stopwatch_wait(miliseconds):
    start = time.time()*1000 # in miliseconds
    time.clock()    
    elapsed = 0
    while elapsed < miliseconds:
      elapsed = time.time()*1000 - start
      stopMotor(4)


def set_x(speed):
    actual=100 if speed>0 else -100
    setMotor(1,actual)
    setMotor(3,actual)

def set_y(speed):
    actual=100 if speed>0 else -100
    print(actual)
    setMotor(2,-actual)
    setMotor(4,actual)

def stop_x():
    stopMotor(1)
    stopMotor(3)
def stop_y():
    stopMotor(2)
    stopMotor(4)

      
def test_ben(a,b,p,t):
  t_on=(p/(a+b))*a
  t_off=(p/(a+b))*b
  print(t_on,t_off)

  for i in range(1,int(t/p)):
    setMotor(1,-100)
    setMotor(2,-100)
    setMotor(3,100)
    setMotor(4,-100)
    
    time.sleep(t_on/1000.0)
    stopMotors()
    time.sleep(t_off/1000.0)

def test_forward(a,b,p,t):
    t_on=(p/(a+b))*a
    t_off=(p/(a+b))*b


    for i in range(1,int(t/p)):
        set_y(100)

        time.sleep(t_on/1000.0)
        stop_y()
        time.sleep(t_off/1000.0)



# motor_move(1,-1)
# setMotor(1,100)
# time.sleep(6)
# stopMotor(1)
stopMotor(1)
'''
for i in range(10):
    # time.sleep(2)
    motor_move(1, 15+i)
    print('speed =',i+15)
    time.sleep(2)
'''
motor_move(1, 18)
time.sleep(5)
motor_move(1,17)
time.sleep(5)
stopMotor(1)




