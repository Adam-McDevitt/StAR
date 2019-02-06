#-------------------------------------------------------------------------pynput
from pynput import keyboard
import threading
import time
from pydispatch import dispatch 
import logging

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-10s) %(message)s',
                    )

trigger = False
stop_all= False
e_press=threading.Event()
e_release=threading.Event()

def wait_for_event_timeout(e, t):
    """Wait t seconds and then timeout"""
    while True:
        event_is_set = e.wait(t)
        if event_is_set:
            trigger=True
            logging.debug('trigger is now:  ',trigger)
        else:
            logging.debug('doing other work')
            
            
def wait_for_event(e):
    """Wait for the event to be set before doing anything"""
    logging.debug('wait_for_event starting')
    event_is_set = e.wait()
    if(event_is_set):
      trigger=!trigger


def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(key.char))
	e_press.set()
	logging.debug('set the event from keypress')
    except AttributeError:
        print('special key {0} pressed'.format(key))

def on_release(key):
    print('{0} released'.format(key))
    e_releas
    if key == keyboard.Key.esc:  
      # Stop listener
      return False
    
# Thread for listener
def listener_key():
    print threading.currentThread().getName(), 'Starting'
    # Collect events until released
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
	listener.join()
    print threading.currentThread().getName(), 'Exiting'

#Thread for the robot
def bobby():
    print threading.currentThread().getName(), 'Starting'
    while(stop_all==False):
      while(trigger):
	print('TRUE')
      
      
    print threading.currentThread().getName(), 'Exiting'

while True:
  wait_for_event(e_press)
  wait_for_event(e_release)
  
# Initiating treads
bobby_thread = threading.Thread(name='bobby', target=bobby)
listener_key_thread = threading.Thread(name='listener_key', target=listener_key)

# Starting threads
bobby_thread.start()
listener_key_thread.start()
#t.start()
#--------------------------------------------------------------------------pynput