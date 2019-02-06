# !/usr/bin/env python

import time
import cv2
from flask import Flask
import threading


# import MotorControl

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

        threading.Thread(target=runFlask).start()

    def control(self):
        print('self.getSensors-->{}\nself.getInputs-->{}'.format(self.getSensors(), self.getInputs()))
        # self.mc.setMotor(1, 100 if self.getSensors()[0] >= 500 else 100)
        # self.sc.__displayDeviceInfo
        # print(self.mc.getInputCount())

        #self.mc.setMotor(1, -100)
        #self.mc.setMotor(2, -100)
        #self.mc.setMotor(3, 100)
        #self.mc.setMotor(5, 100)
        # self.mc.setMotor(5,100)
        # self.mc.setMotor(2, 100 if self.getSensors()[0] >= 500 else -100)
        # self.mc.setMotor(3, 100 if self.getSensors()[0] >= 500 else -100)
        # self.mc.setMotor(4, 100 if self.getSensors()[0] >= 500 else -100)
        # self.mc.setMotor(5, 100 if self.getSensors()[0] >= 500 else -100)
        # self.mc.setMotor(6, 100 if self.getSensors()[0] >= 500 else -100)
        print('!!!!!OUTSIDE!!!!!!!!!')
        if not self.timerIsSet:
            time_end = time.time() + 10
            time_start = time.time() + 2
        self.timerIsSet = True
        if time.time() >= time_start and time.time() <= time_end:
            print('???????????INSIDE????????')
            self.mc.setMotor(1, -100)
            self.mc.setMotor(2, -100)

        self.sc.engage()
        self.sc.setPosition(0 if self.getSensors()[0] >= 500 else 180)

        time.sleep(0.5)

    def vision(self):
        image = self.camera.getFrame()
        self.camera.imshow('Camera', image)


# Flask stuff


app = Flask(__name__)


def runFlask():
    app.run(host='0.0.0.0', threaded=True)


@app.route('/')
def home():
    return "Server is working!"
