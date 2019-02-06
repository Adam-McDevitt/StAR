# !/usr/bin/env python

import time
import cv2
import os
import subprocess
import server
import multiprocessing
import curses
import threading
import logging


#####################################################################
#			GLOBALS
#####################################################################

#Movement commands
forward = False
backwards = False
right_rotation  = False
left_rotation = False

#Siganls received
sig1 = 0
sig2 = 0
sig3 = 0
sig4 = 0
######################################################################

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-10s) %(message)s',
                    )


class Toddler:
    __version = '2018a'

    timerIsSet = False

    def __init__(self, IO):
        print('[Toddler] I am toddler {} playing in a sandbox'.format(Toddler.__version))

        self.camera = IO.camera.initCamera('pi', 'low')
        self.getInputs = IO.interface_kit.getInputs
        self.getSensors = IO.interface_kit.getSensors
        self.mc = IO.motor_control
        self.sc = IO.servo_control
        #subprocess.call("python server.py", shell=True)

        #p = multiprocessing.Process(target=server.run)
        #p.start()

    def control(self):
	print('entering')
        #print('self.getSensors-->{}\nself.getInputs-->{}'.format(self.getSensors(), self.getInputs()))
        # self.mc.setMotor(1, 100 if self.getSensors()[0] >= 500 else 100)
        # self.sc.__displayDeviceInfo
        # print(self.mc.getInputCount())
  
	#self.mc.setMotor(4,100)
	
	my_var = 0
	print('\n',my_var)
	my_var +=1


        #Ilie code----------------------------------------------------------
	'''
	key = ''
	stdscr=curses.initscr()
	curses.noecho()
	curses.halfdelay(1)
	#stdscr.nodelay(1)
	stdscr.keypad(True)
	#try:    
	
        while (True):
	  #print('\nINSIDE')
	  #key = cv2.waitKey(1) & 0xFF
	
	  #print('key',key)
	
	  # if key == 27: #ESC
	  #   break

	  #win = curses.newwin(height, width, begin_y, begin_x)
	  key = stdscr.getch()
	  self.mc.setMotor(1,10)
	  self.mc.setMotor(2,10)
	  self.mc.setMotor(3,10)
	  self.mc.setMotor(4,10)
	  if key == curses.KEY_UP:
	      print('Forward')
	      self.mc.setMotor(1,-100)
	      self.mc.setMotor(2,-100)
	      self.mc.setMotor(3,100)
	      self.mc.setMotor(4,100)
	      key=''
	  elif key == curses.KEY_DOWN:
	      print('Backwards')
	      self.mc.setMotor(1,100)
	      self.mc.setMotor(2,100)
	      self.mc.setMotor(3,-100)
	      self.mc.setMotor(4,-100)
	      key=''
	  if key == curses.KEY_LEFT:
	      print('LEFT')
	      self.mc.setMotor(1,10)
	      self.mc.setMotor(2,10)
	      self.mc.setMotor(3,10)
	      self.mc.setMotor(4,10)
	      key=''
	  elif key == curses.KEY_RIGHT:
	      print('RIGHT')
	      self.mc.setMotor(1,10)
	      self.mc.setMotor(2,10)
	      self.mc.setMotor(3,10)
	      self.mc.setMotor(4,10)
	      key=''
	  else:
	      self.mc.setMotor(1,10)
	      self.mc.setMotor(2,10)
	      self.mc.setMotor(3,10)
	      self.mc.setMotor(4,10)
	      key=''
	  try:
	    key=stdscr.getch()
	  except:
	    key==ord('q')
	  #finally:
	   #   curses.halfdelay(3)
	    #  stdscr.nodelay(1)
	      #stdscr.keypad(0)
	     # curses.noecho()
	      #curses.endwin()
	#Ilie code----------------------------------------------------------
	'''
	
	
	
	#Ilie code----------------------------------------------------------
	
	
	
        #self.sc.engage()
        #self.sc.setPosition(0 if self.getSensors()[0] >= 500 else 180)

        time.sleep(0.5)
        print('exiting control()')

    def vision(self):
	print('Entering camera')
        image = self.camera.getFrame()
        self.camera.imshow('Camera', image)
        print('Exiting camera')
    #def siganl_sim():
      
