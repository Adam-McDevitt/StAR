import threading
from pynput import keyboard
import logging
import time
import numpy as np
import smbus

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-10s) %(message)s',
                    )


cond_motor=threading.Condition()
motor_signal=0
motor_direction_signal=0
motor_signal_stop=True
sleep_delay=0.001

#---------------------------------------timer function
def timer_run(seconds_run):
    global motor_signal
    global motor_signal_stop
    global sleep_delay
    start_time=time.time()
    time.clock() #clock starts ticking
    elapsed = 0
    while elapsed < seconds_run:
      elapsed = time.time() - start_time
      try:
	cond_motor.acquire()
	if (motor_signal==0):
	  motor_signal=1
	  cond_motor.notify_all()
	  time.sleep(sleep_delay)
	elif(motor_signal==1):
	  motor_signal=2
	  cond_motor.notify_all()
	else:
	  cond_motor.release()
      except AttributeError:
	print('except')
    if(elapsed>=seconds_run):
      try:
	cond_motor.acquire()
	motor_signal=0
	cond_motor.notify_all()
      except AttributeError:
	print('except2')
#---------------------------------------timer function

#---------------------------------------timer function
def timer_wait(seconds_wait,seconds_run):
    global motor_direction_signal 
    global cond_motor
    global sleep_delay
    global motor_signal_stop
    start_time=time.time()
    time.clock()
    elapsed = 0
    while elapsed < seconds_wait:
      elapsed = time.time() - start_time
    if(elapsed>=seconds_wait):
      
      #FORWARD
      try:
	cond_motor.acquire()
	motor_direction_signal=1
	cond_motor.notify_all()
      except AttributeError:
	print('except2')
      timer_run(seconds_run)
      
      time.sleep(sleep_delay)
    
      
      #BACKWARD
      try:
	cond_motor.acquire()
	motor_direction_signal=2
	cond_motor.notify_all()
      except AttributeError:
	print('except2')
      time.sleep(sleep_delay)
      timer_run(seconds_run)
      
      time.sleep(sleep_delay)
      
      
      #LEFT MOVE
      try:
	cond_motor.acquire()
	motor_direction_signal=3
	cond_motor.notify_all()
      except AttributeError:
	print('except2')
      time.sleep(sleep_delay)
      timer_run(seconds_run)
      
      time.sleep(sleep_delay)
      
      #RIGHT MOVE
      try:
	cond_motor.acquire()
	motor_direction_signal=4
	cond_motor.notify_all()
      except AttributeError:
	print('except2')
      time.sleep(sleep_delay)	
      timer_run(seconds_run)
      
      time.sleep(sleep_delay)
      
      #LEFT ROTATION
      try:
	cond_motor.acquire()
	motor_direction_signal=5
	cond_motor.notify_all()
      except AttributeError:
	print('except2')
      time.sleep(sleep_delay)	
      timer_run(seconds_run)
      
      time.sleep(sleep_delay)      
      
      #RIGHT ROTATION
      try:
	cond_motor.acquire()
	motor_direction_signal=6
	cond_motor.notify_all()
      except AttributeError:
	print('except2')
      time.sleep(sleep_delay)	
      timer_run(seconds_run)
      
      time.sleep(sleep_delay)
      
      #STOP THE WHOLE THING
      try:
	cond_motor.acquire()
	motor_signal_stop=False
	cond_motor.notify_all()
      except AttributeError:
	print('except2')   
      time.sleep(sleep_delay)
      
#---------------------------------------timer function


#------------------------------------------------------------------MOTORS CODE
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


#-----------------------------------------------------------------------MOTORS CODE

    
    
class SharingSignals(threading.Thread):
  global motor_signal
  global motor_signal_stop 
  def __init__(self, type):
    threading.Thread.__init__(self)
    self.type=type

      
  def run(self):
    global motor_signal
    global motor_signal_stop
    global motor_direction_signal
    global sleep_delay
    if(self.type==1): 
	timer_wait(3,1)
    elif(self.type==2):
        while(motor_signal_stop):
            if (motor_signal==1):
		  if(motor_direction_signal==1): #FORWARD
		    setMotor(2,-100)
		    setMotor(4,100)
		  elif(motor_direction_signal==2): #BACKWARD
		    setMotor(2,100)
		    setMotor(4,-100) 
		  elif(motor_direction_signal==3): #LEFT MOVE
		    setMotor(1,-100)
		    setMotor(3,-100)	
		  elif(motor_direction_signal==4): #RIGHT MOVE
		    setMotor(1,100)
		    setMotor(3,100)	
		  elif(motor_direction_signal==5): #LEFT ROTATION
		    setMotor(1,-100)
		    setMotor(2,-100)
		    setMotor(3,100)
		    setMotor(4,-100)
		  elif(motor_direction_signal==6): #RIGHT ROTATION
		    setMotor(1,100)
		    setMotor(2,100)
		    setMotor(3,-100)
		    setMotor(4,100)	
		  time.sleep(sleep_delay)
            elif(motor_signal==0):
                stopMotors()
                
                time.sleep(sleep_delay)
      

thread1 = SharingSignals(1) ##waiting for Flask signal to trigger waiting and running timers
thread2 = SharingSignals(2) #waits for motor signals to start or stop the motors
thread2.start()
thread1.start() 
thread1.join() #Wait for the thread to finish
thread2.join() #Wait for the thread to finish 

 












