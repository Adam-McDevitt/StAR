import serial
import sys
import time




ser = serial.Serial('/dev/ttyACM0')


while 1:
    if (ser.in_waiting > 0):
        print(ser.readline())
        





