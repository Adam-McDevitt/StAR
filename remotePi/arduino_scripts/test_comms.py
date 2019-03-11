import serial

from read_ports import serial_ports


ser=serial.Serial(serial_ports()[0],9600)
print(ser)
while True :
    line=ser.readline()
    reading=line.split('\t')
    diff=reading[-1]
    print (diff)
