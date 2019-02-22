import numpy as np
import smbus
import time
import requests
import time
import server
from multiprocessing import Process, Manager, Lock

class Robot:
    #def __init__(self,bus,address,movement):
    #    self.bus = bus
    #    self.address = address
    #    self.movement = movement

    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.address = 0x04
        self.movement = ''

    #---------------------------------------------------------------------------
    #                              Useful functions
    #---------------------------------------------------------------------------

    # Writing to motorboard
    def motor_move(self,id, power):
        """
        Smooth control of the motors
        Mode 2 is Forward.
        Mode 3 is Backwards.
        """
        mode = 2 if power >= 0 else 3
        cmd_byte = id << 5 | 24 | mode << 1
        pwr = int(abs(power) * 2.55)
        self.bus.write_i2c_block_data(self.address, 0, [cmd_byte, pwr])

    def write(self,value):
        self.bus.write_byte_data(self.address, 0x00, value)

    def stopMotor(self,id):
        """
        Mode 0 floats the motor.
        """
        direction = 0
        byte1 = id << 5 | 16 | direction << 1
        self.write(byte1)


    def stopMotors(self):
        """
        The motor board stops all motors if bit 0 is high.
        """
        self.write(0x01)

    # Gradual Movement function
    def gradual_start(self,dir, time_move):
        print('Starting motors')
        time_inc = 0.2
        if dir == 'FORWARD':
            print('FORWARD')
            for i in range(1,6):
                print(i,i*20)
                self.motor_move(2,-20*i)
                self.motor_move(4, 20*i)
                time.sleep(time_inc)
                if i == 5:
                    print('Got to top speed')
                    self.keep_moving(dir, (time_move-(5*time_inc)))
        elif(dir == 'BACKWARD'):
            print('BACKWARD')
            for i in range(1,6):
                self.motor_move(2, 20*i)
                self.motor_move(4,-20*i)
                time.sleep(time_inc)
                print(i,i*20)
                if i == 5:
                    print('Got to top speed')
                    self.keep_moving(dir, (time_move-5*time_inc))
        elif(dir == 'RIGHT'):
            print('RIGHT')
            for i in range(1,6):
                self.motor_move(1,20*i)
                self.motor_move(3,20*i)
                time.sleep(time_inc)
                if i == 5:
                    print('Got to top speed')
                    self.keep_moving(dir, (time_move-5*time_inc))
        elif(dir == 'LEFT'):
            print('LEFT')
            for i in range(1,6):
                self.motor_move(1,-20*i)
                self.motor_move(3,-20*i)
                time.sleep(time_inc)
                if i == 5:
                    print('Got to top speed')
                    self.keep_moving(dir, (time_move-5*time_inc))

    # Keep moving
    def keep_moving(self,dir, time_left):
        keep_moving_delay = 0.1
        print('time_left=',time_left)
        if dir == 'FORWARD':
            print('Moving FORWARD')
            self.motor_move(2,-100)
            self.motor_move(4, 100)
            time.sleep(time_left)
            self.stopMotors()
            time.sleep(keep_moving_delay)
        elif dir == 'BACKWARD':
            print('Moving BACKWARD')
            self.motor_move(2, 100)
            self.motor_move(4,-100)
            time.sleep(time_left)
            self.stopMotors()
            print('Shuting down motors')
            time.sleep(keep_moving_delay)
        elif dir == 'RIGHT':
            print('Moving RIGHT')
            self.motor_move(1,100)
            self.motor_move(3,100)
            time.sleep(time_left)
            self.stopMotors()
            time.sleep(keep_moving_delay)
        elif dir == 'LEFT':
            print('Moving LEFT')
            self.motor_move(1,-100)
            self.motor_move(3,-100)
            time.sleep(time_left)
            self.stopMotors()
            time.sleep(keep_moving_delay)
        else:
            print('COMMAND NOT RECOGNIZED')
        print('Done moving')
        time.sleep(keep_moving_delay)

    def quick_rotation(self,direction, time_move):
        if(direction == 'RIGHT'):
            print('Right rotation')
            self.motor_move(1,-100)
            self.motor_move(2,-100)
            self.motor_move(3, 100)
            self.motor_move(4,-100)
            time.sleep(time_move)
            self.stopMotors()
        elif(direction == 'LEFT'):
            print('LEFT rotation')
            self.motor_move(1, 100)
            self.motor_move(2, 100)
            self.motor_move(3,-100)
            self.motor_move(4, 100)
            time.sleep(time_move)
            self.stopMotors()
        else:
            print('COMMAND NOT RECOGNIZED')
        print('Done quick rotation')
    def move_diagonal(self,dir, time_move):
        if dir == 'RIGHTUP':
            self.motor_move(2,-100)
            self.motor_move(4, 100)
            self.motor_move(1, 100)
            self.motor_move(3, 100)
            time.sleep(time_move)
            self.stopMotors()
        elif dir == 'RIGHTDOWN':
            self.motor_move(2, 100)
            self.motor_move(4,-100)
            self.motor_move(1, 100)
            self.motor_move(3, 100)
            time.sleep(time_move)
            self.stopMotors()
        elif dir == 'LEFTUP':
            self.motor_move(2,-100)
            self.motor_move(4, 100)
            self.motor_move(1,-100)
            self.motor_move(3,-100)
            time.sleep(time_move)
            self.stopMotors()
        elif dir == 'LEFTDOWN':
            self.motor_move(2, 100)
            self.motor_move(4,-100)
            self.motor_move(1,-100)
            self.motor_move(3,-100)
            time.sleep(time_move)
            self.stopMotors()
        else:
            print('COMMAND NOT RECOGNIZED')
            self.stopMotors()

    #-----------------------------------------------------------------------------
    def write(self,value):
        self.bus.write_byte_data(self.address, 0x00, value)


    def setMotor(self,id, speed):
        """
        Mode 2 is Forward.
        Mode 3 is Backwards.
        """
        direction = 2 if speed >= 0 else 3
        speed = np.clip(abs(speed), 0, 100)
        byte1 = id << 5 | 24 | direction << 1
        byte2 = int(speed * 2.55)
        self.write(byte1)
        self.write(byte2)


    def stopMotor(self,id):
        """
        Mode 0 floats the motor.
        """
        direction = 0
        byte1 = id << 5 | 16 | direction << 1
        self.write(byte1)


    def stopMotors(self):
        """
        The motor board stops all motors if bit 0 is high.
        """
        self.write(0x01)

    def stopwatch_run(self,miliseconds):
        start = time.time()*1000 # in miliseconds
        time.clock()
        elapsed = 0
        while elapsed < miliseconds:
          elapsed = time.time()*1000 - start
          self.setMotor(4,70)

    def stopwatch_wait(self,miliseconds):
        start = time.time()*1000 # in miliseconds
        time.clock()
        elapsed = 0
        while elapsed < miliseconds:
          elapsed = time.time()*1000 - start
          self.stopMotor(4)


    def set_x(self,speed):
        actual=100 if speed>0 else -100
        self.setMotor(1,actual)
        self.setMotor(3,actual)

    def set_y(self,speed):
        actual=100 if speed>0 else -100
        print(actual)
        self.setMotor(2,-actual)
        self.setMotor(4,actual)

    def stop_x(self):
        self.stopMotor(1)
        self.stopMotor(3)
    def stop_y(self):
        self.stopMotor(2)
        self.stopMotor(4)


    def test_ben(self,a,b,p,t):
      t_on=(p/(a+b))*a
      t_off=(p/(a+b))*b
      print(t_on,t_off)

      for i in range(1,int(t/p)):
        self.setMotor(1,-100)
        self.setMotor(2,-100)
        self.setMotor(3,100)
        self.setMotor(4,-100)

        time.sleep(t_on/1000.0)
        self.stopMotors()
        time.sleep(t_off/1000.0)

    def test_forward(self,a,b,p,t):
        t_on=(p/(a+b))*a
        t_off=(p/(a+b))*b


        for i in range(1,int(t/p)):
            self.set_y(100)

            time.sleep(t_on/1000.0)
            self.stop_y()
            time.sleep(t_off/1000.0)

    def variable_speed(self,direction,a,b,p,t):
        t_on=(p/(a+b))*a
        t_off=(p/(a+b))*b


        if direction[0]=="y":
            for i in range(1,int(t/p)):
                if(direction[1]=="+"):
                    self.set_y(100)
                else:
                    self.set_y(-100)

                time.sleep(t_on/1000.0)
                self.stop_y()
                time.sleep(t_off/1000.0)
        else:
            for i in range(1,int(t/p)):
                if(direction[1]=="+"):
                    self.set_x(100)
                else:
                    self.set_x(-100)

                time.sleep(t_on/1000.0)
                self.stop_x()
                time.sleep(t_off/1000.0)

if __name__=="__main__":
        val=1
        flag=0

        drive_time=2000
        wait_time=1

        a=9
        b=1

        Makis=Robot()
        Makis.gradual_start('FORWARD',2)
        Makis.stopMotors()
