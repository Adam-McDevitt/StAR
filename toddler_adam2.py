# !/usr/bin/env python

import time
import cv2
import os
import subprocess
import server
from multiprocessing import Process, Value, Array, Manager
import thread


# import MotorControl

class Toddler:
    __version = '2018a'

    # Updated with data that is used by the server. Data is sent to the app upon request. Data could include
    # number of packages delivered so far, currently speed, problems, notifications, etc.

    def __init__(self, IO):
        print('[Toddler] I am toddler {} playing in a sandbox'.format(Toddler.__version))

        self.camera = IO.camera.initCamera('pi', 'low')
        self.getInputs = IO.interface_kit.getInputs
        self.getSensors = IO.interface_kit.getSensors
        self.mc = IO.motor_control
        self.sc = IO.servo_control
        # subprocess.call("python server.py", shell=True)

        # Currently unsure whether to use processes or threads

        self.dictionary = Manager().dict()
        self.count = 0

        process = Process(target=server.run, args=[self.dictionary])
        process.start()

        #thread.start_new_thread(server.run, args=self.dictionary)

    def control(self):
        print('self.getSensors-->{}\nself.getInputs--> {}'.format(self.getSensors(), self.getInputs()))
        # self.mc.setMotor(1, 100 if self.getSensors()[0] >= 500 else 100)
        # self.sc.__displayDeviceInfo
        # print(self.mc.getInputCount())

        #self.mc.setMotor(1, 100)

        self.sc.engage()
        self.sc.setPosition(0 if self.getSensors()[0] >= 500 else 180)

        time.sleep(0.5)
        self.count = self.count + 1

        self.dictionary['count'] = str(self.count)


    def vision(self):
        image = self.camera.getFrame()
        self.camera.imshow('Camera', image)
