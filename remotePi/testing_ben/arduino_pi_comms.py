from __future__ import print_function
import time
import serial
# import server
import requests
from multiprocessing import Process


ser = serial.Serial('/dev/ttyACM0')
if __name__ == "__main__":

    # is_executing_plan = False
    # time.sleep(5)
    # while 1:
    #     print("top of loop")
    #     if (not is_executing_plan):
    #         print("in not executing plan")
    #         r = requests.get('http://0.0.0.0:5000/getinstructions')
    #         if (r.text != "0"):
    #             ser.write(r.text.encode())
    #             is_executing_plan = True
    #     elif (ser.in_waiting > 0):
    #         print("in  executing plan")
    #         line = ser.readline().strip()
    #         if (line == "done"):
    #             is_executing_plan = False
    #         else:
    #             requests.get('http://0.0.0.0:5000/addnotification?not=' + line)
    print("entering loop")
    while 1:
        if (ser.in_waiting>0):
            print(ser.read(),end= " ")
        