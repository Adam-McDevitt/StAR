from __future__ import print_function
import time
import serial
# import server
import requests
from multiprocessing import Process


if __name__ == "__main__":


    while (1):
        try:
            print("trying")
            ser = serial.Serial('/dev/ttyACM0',baudrate=115200)
            time.sleep(2)
            print("entering loop")
            while 1:
                if (ser.in_waiting>0):
                    print(ser.readline(),end= " ")
        except:
            pass
        