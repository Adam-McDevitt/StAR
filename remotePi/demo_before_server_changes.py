import numpy as np
import smbus
import time
import requests
import time

bus = smbus.SMBus(1)
address = 0x04


def write(value):
    bus.write_byte_data(address, 0x00, value)


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

def variable_speed(direction,a,b,p,t):
    t_on=(p/(a+b))*a
    t_off=(p/(a+b))*b


    if direction[0]=="y":
        for i in range(1,int(t/p)):
            if(direction[1]=="+"):
                set_y(100)
            else:
                set_y(-100)

            time.sleep(t_on/1000.0)
            stop_y()
            time.sleep(t_off/1000.0)
    else:
        for i in range(1,int(t/p)):
            if(direction[1]=="+"):
                set_x(100)
            else:
                set_x(-100)

            time.sleep(t_on/1000.0)
            stop_x()
            time.sleep(t_off/1000.0)



if __name__=="__main__":
    val=1
    flag=0

    drive_time=4000
    wait_time=1

    a=9
    b=1

    while True:

        print(val)
        try:
            print("contacting server")
            r = requests.get('http://leconte:5000/getinstructions')

            if int(r.text)==1:
                continue
            elif int(r.text)==2 and flag==0:
                # demo code
                variable_speed("y+",a,b,150,drive_time)
                time.sleep(drive_time)
                stop_y()
                time.sleep(wait_time)
                variable_speed("x-",a,b,250,drive_time)
                time.sleep(drive_time)
                stop_x()
                time.sleep(wait_time)
                variable_speed("y-",a,b,350,drive_time)
                time.sleep(drive_time)
                stop_y()
                time.sleep(wait_time)
                variable_speed("x+",a,b,500,drive_time)
                time.sleep(drive_time)
                stop_x()
                time.sleep(wait_time)







                flag=1
            elif int(r.text)==3:
                flag=0
                val=1

        except Exception as e:
            print(e)
            print()
            print("will retry in 5")
            time.sleep(5)    


