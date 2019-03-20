import serial
from time import sleep
from read_ports import serial_ports
ser=serial.Serial(serial_ports()[0],9600)

while True :
    try:
                    # print("contacting server")
                    r = requests.get('http://0.0.0.0:5000/getinstructions')
                    # print('request is: {}', int(r.text))
                    if(int(r.text) == 11):
                        user_input=str.encode('1')
                    elif(int(r.text) == 12):
                        user_input=str.encode('2')
    except:
        print("WTF")
    ser.write(user_input) # Convert the decimal number to ASCII then send it to the Arduino
    sleep(0.1)