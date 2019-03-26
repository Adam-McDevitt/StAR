import serial
import sys
import time

ser = serial.Serial('/dev/ttyACM0')
inputs = sys.argv
num_desired_inputs = 16
inputToSend = ""
if len(inputs) == num_desired_inputs + 1:

    time.sleep(5)

    print("kp0 = " + inputs[1])
    print("kp1 = " + inputs[2])
    print("kp2 = " + inputs[3])
    print("sp3 = " + inputs[4])

    print("kd0 = " + inputs[5])
    print("kd1 = " + inputs[6])
    print("kd2 = " + inputs[7])
    print("kd3 = " + inputs[8])

    print("ki0 = " + inputs[9])
    print("ki1 = " + inputs[10])
    print("ki2 = " + inputs[11])
    print("ki3 = " + inputs[12])

    print("sp0 = " + inputs[13])
    print("sp1 = " + inputs[14])
    print("sp2 = " + inputs[15])
    print("sp3 = " + inputs[16])

    inputToSend = inputs[1]
    for i in range(2, num_desired_inputs + 1):
        inputToSend = inputToSend + " " + inputs[i]
    print("writing " + inputToSend)
    ser.write(inputToSend.encode())
    print("wrote " + inputToSend.encode() + " encoded")
else:
    print("Inputs must be formatted as 16 floats representing the constants, i.e.:\n" +
          "python arduinodebug.py kp0 kp1 kp2 kp3 kd0 kd1 kd2 kd3 ki0 ki1 ki2 ki3 sp0 sp1 sp2 sp3 ")
    exit()
f = open("results" + inputToSend.replace(" ","-") + ".csv","w+")
while 1:
    if (ser.in_waiting > 0):
        line = ser.readline().strip()
        if line != "done":
            f.write(line + "\n")
            print(line)
        else:
            print("exit")
            f.close()
            exit()
