#include <Wire.h>
#include <SDPArduino.h>
#include <MotorPIDs.h>

#define ROTARY_SLAVE_ADDRESS 5;




MotorPIDs* test=new MotorPIDs();


void setup(){
  
  SDPsetup();
  delay(100);
  Serial.println("helloooooo");
  delay(100);
  test->MotorPIDsinit();
  delay(2000);
  test->setup_PID(4,18,0.6,1);//motor,Kp,Ki,Kd
  test->setup_PID(2,18,0.6,1);//motor,Kp,Ki,Kd
//  test->setup_PID(1,20,0.8,2);//motor,Kp,Ki,Kd
//  test->setup_PID(3,20,0.8,2);//motor,Kp,Ki,Kd
  motorAllStop();
}

void loop(){

  test->setSetpoint(2,-3);
  test->setSetpoint(4,-3);
//   test->setSetpoint(1,-1.5);
//  test->setSetpoint(3,1.5);
  test->update_motor_PID(2);
  test->update_motor_PID(4);
//  test->update_motor_PID(1);
//  test->update_motor_PID(3);
  int i=0;
  Serial.println("POWERS");

    for(i=0;i<6;i++){
    float val=test->motor_pows[i];
    Serial.print(val);
    Serial.print(' ');
    }
  Serial.println();
  test->MotorPIDs::printVelocities();
    Serial.println();

 }
