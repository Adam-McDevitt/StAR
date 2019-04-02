import time
import serial
import server
import requests
from multiprocessing import Process
from read_ports import serial_ports

ser=serial.Serial(serial_ports()[0],115200)
START = 7
DONE = 0


def shelfarray(text):
    pspairs = text.split(';')  # package-shelf-pairs
    instructions = [START]
    for i in range(0, len(pspairs)):
        pair = pspairs[i].split(',')
        package = pair[0]
        shelf = pair[1]
        instructions = instructions + [int(shelf)]
    instructions = instructions + [DONE]
    print instructions
    return instructions


if __name__ == "__main__":
    process = Process(target=server.run)
    process.start()
    is_executing_plan = False
    time.sleep(5)
    while 1:
        if (not is_executing_plan):
            r = requests.get('http://0.0.0.0:5000/getinstructions')
            if (r.text != "0"):
                instructions = shelfarray(r.text.strip())
                ser.write(bytearray(instructions))
                is_executing_plan = True
        elif (ser.in_waiting > 0):
            line = ser.readline().strip()
            if (line == "done"):
                is_executing_plan = False
            else:
                requests.get('http://0.0.0.0:5000/addnotification?not=' + line)
