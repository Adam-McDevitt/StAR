import numpy as np
import smbus
import time

bus = smbus.SMBus(1)
address = 0x04


def write(value):
    bus.write_byte_data(address, 0x00, value)


def setMotor(id, speed):
    """
    Mode 2 is Forward.
    Mode 3 is Backwards.
    """
    direction = 2 if speed >= 0 else 3
    
    motor_mode = 0x2
    
    speed = np.clip(abs(speed), 0, 100)
    byte1 = id << 5 | 24 | direction << 1
    byte2 = int(speed * 2.55)
    write(byte1)
    write(byte2)


def stopMotor(id):
    """
    Mode 0 floats the motor.
    """
    direction = 0
    byte1 = id << 5 | 16 | direction << 1
    write(byte1)


def stopMotors():
    """
    The motor board stops all motors if bit 0 is high.
    """
    write(0x01)


setMotor(2, 100)
setMotor(4, 100)
time.sleep(2)

stopMotors()
# stopMotor(4)
time.sleep(0.5)

setMotor(2, -100)
time.sleep(1)

setMotor(4, -100)
time.sleep(1)

stopMotors()
