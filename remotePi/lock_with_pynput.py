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
motor_signal_stop=True
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

def on_press(key):
    global motor_signal
    global motor_signal_stop
    try:
	#print('alphanumeric key {0} pressed'.format(key.char))
	cond_motor.acquire()
	if(motor_signal==0):
	    motor_signal=1
	    cond_motor.notify_all()
	elif(motor_signal==1):      
	    motor_signal=2
	    cond_motor.notify_all()
	    #logging.debug('set motor_signal to '+str(motor_signal))
	else:
	    cond_motor.release()
    except AttributeError:
	print('special key {0} pressed'.format(key))

def on_release(key):
    global motor_signal
    global motor_signal_stop
    #print('alphanumeric key {0} released'.format(key.char))
    cond_motor.acquire()
    if(motor_signal==1 or motor_signal==2):      
	    motor_signal=0
	    cond_motor.notify_all()
	    #logging.debug('set motor_signal to '+str(motor_signal))
    else:
	    cond_motor.release()
    if key == keyboard.Key.esc:  
      # Stop listener
      return False
    
    
class SharingSignals(threading.Thread):
  global motor_signal
  global motor_signal_stop    
  def __init__(self, type):
    threading.Thread.__init__(self)
    self.type=type

      
  def run(self):
    global motor_signal
    global motor_signal_stop
    if(self.type==1):
	    #print threading.currentThread().getName(), 'Starting'
	    # Collect events until released
	    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
	      listener.join()
	    #print threading.currentThread().getName(), 'Exiting'
	    motor_signal_stop=False  
	    # Thread for listener
    elif(self.type==2):
        while(motor_signal_stop):
            if (motor_signal==1):
                setMotor(2,-100)
                setMotor(4,100)
                time.sleep(0.01)
            elif(motor_signal==0):
                stopMotors()
                time.sleep(0.01)

      
thread1 = SharingSignals(1)
thread2 = SharingSignals(2)
thread2.start()
thread1.start() 
thread1.join() #Wait for the thread to finish
thread2.join() #Wait for the thread to finish 

 












