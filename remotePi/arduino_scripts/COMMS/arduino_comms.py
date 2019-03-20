import serial
import server
from time import sleep
from read_ports import serial_ports
from multiprocessing import Process, Manager, Lock
import requests


if __name__== "__main__":
    user_input=""
    process = Process(target=server.run)
    process.start()
    ser=serial.Serial(serial_ports()[0],115200,timeout=1)
    while True :
        try:
                        # print("contacting server")
                        r = requests.get('http://0.0.0.0:5000/getinstructions')
                        print("INST IS ",int(r.text))
                        # print('request is: {}', int(r.text))
                        if(int(r.text) == 11):
                            user_input=str.encode('1')
                            
                        elif(int(r.text) == 12):
                            user_input=str.encode('2')
        except:
            print("WTF")
        if(user_input==str.encode('1') or user_input==str.encode('2')):
            print("I'M WRITING", user_input)
            ser.write(user_input) # Convert the decimal number to ASCII then send it to the Arduino
            sleep(0.1)
            
        else:
            print("NO INSTRUCTION")