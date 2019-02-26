import serial
ser = serial.Serial('/dev/ttyACM2',9600)
while True :
    line=ser.readline()
    reading=line.split('\t')
    diff=reading[-1]
    print diff
