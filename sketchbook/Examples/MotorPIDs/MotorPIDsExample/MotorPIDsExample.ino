#include <Wire.h>
#include <SDPArduino.h>
#include <MotorPIDs.h>

#define ROTARY_SLAVE_ADDRESS 5;
#define PAST 150;




MotorPIDs* test=new MotorPIDs();


void setup(){
  
  SDPsetup();
  delay(100);
  Serial.println("helloooooo");
  delay(100);
  test->MotorPIDsinit();
  delay(2000);
  test->setup_PID(1,20,0.8,2);//motor,Kp,Ki,Kd
  test->setup_PID(3,20,0.8,2);//motor,Kp,Ki,Kd
}

void loop(){

  test->setSetpoint(1,4.5);
  test->setSetpoint(3,4.5);
  test->update_motor_PID(1);
  test->update_motor_PID(3);
  int i=0;
  Serial.println("POWERS");

  while (i < MotorPIDs::ROTARY_COUNT){
    float val=test->motor_pows[i];
    float vel=test->velocities[i];
    Serial.print(val);
    Serial.println('-------------------------------------------- ');
    Serial.print(vel);
    Serial.print(' ');
    i++;
  }

  Serial.println();
//  delay(200);
 }



