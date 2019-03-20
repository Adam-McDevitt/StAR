import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(5)
ser.write('test')
