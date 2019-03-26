#include <SDPArduino.h>
#include <Wire.h>
#include<Arduino.h>
#include <MotorPIDs.h>


double MotorPIDs::update_past(int motor_id,double new_error){
 
 double sum=0;
 for(int i=0;i<PAST;i++){
   sum+=pasts[motor_id][i];
 }
 pasts[motor_id][past_counters[motor_id]]=new_error;
 past_counters[motor_id]= (past_counters[motor_id]+1)%PAST;
 return sum;
 
 }

void MotorPIDs::MotorPIDsinit(){
   //velocities for all motors initialized to zero
        for(int i=0; i < ROTARY_COUNT;i++){
          MotorPIDs::velocities[i]=0;
        }

        //PID values for all motors initialized to zero
        for(int i=0;i < ROTARY_COUNT;i++){
          MotorPIDs::motor_pows[i]=0;
          MotorPIDs::vels_deltas_new[i]=0;
          MotorPIDs::vels_deltas_old[i]=0;
        }

        //CALLBACK values initialized to zero(also used as PID values)
        for(int i=0;i < ROTARY_COUNT;i++){
          MotorPIDs::vels_desired[i]=0;
        }
        // for(int i=0;i < ROTARY_COUNT;i++){
        //   MotorPIDs::Kps[i]=0;
        // }
        // MotorPIDs::Kis[i]=0;
        // MotorPIDs::Kds[i]=0;
        // MotorPIDs::directions[i]=0;

        //Ki past entries
        // for(int i=0;i < ROTARY_COUNT;i++){
        //   for(int j=0;j<PAST;j++){
        //     MotorPIDs::pasts[i][j]=0;//2d array, can't initialize without for-loop, but C++ initializes to zero by default(check TODO)
        //   }
        // }
      
        // for(int i=0;i < ROTARY_COUNT;i++){
        //   MotorPIDs::past_counters[i]=0;
        // }
        delay(500);
}

void MotorPIDs::setup_PID(int motor_id, double Kp, double Ki, double Kd){
    
    Kps[motor_id]=Kp;
    Kis[motor_id]=Ki;
    Kds[motor_id]=Kd;
    vels_deltas_old[motor_id]=0.0;
    
    
    /*
    Serial.print("SETUP PID for motor [ ");
    Serial.print(motor_id);
    Serial.print(" ]: ");

    
    Serial.print("Kp: ");
    Serial.print(Kps[motor_id]);
    Serial.print(" ");

    Serial.print("Ki: ");
    Serial.print(Kis[motor_id]);
    Serial.print(" ");

    Serial.print("Kd: ");
    Serial.print(Kds[motor_id]);
    Serial.print(" ");
    */
}

//TODO: SEE IF setDirection CAN BE IN A CALLBACK (Pieris doesn't think so because: (see next lines)
/* 
callbacks are just function names in variables, this is actually a real time change
of the value of direction which will be accessed by updatePID very fast all the time)
*/
void MotorPIDs::setSetpointAndDirection(int motor_id, double vel_desired,int direction){
    directions[motor_id]=direction;
    vels_desired[motor_id]=vel_desired;

    /*
    Serial.print("Setpoint Velocity: ");
    Serial.println(vels_desired[motor_id]);
    */
}


void MotorPIDs::update_motor_PID(int motor_id) {
  // Update motor speeds
  MotorPIDs::update_velocities();
  // calcualte new power
  vels_deltas_new[motor_id] = vels_desired[motor_id] - velocities[motor_id];
  double vel_adjustment = Kps[motor_id] * vels_deltas_new[motor_id] + Kds[motor_id] * (vels_deltas_new[motor_id] - vels_deltas_old[motor_id]) + Kis[motor_id] *MotorPIDs::update_past(motor_id,vels_deltas_new[motor_id]);
  vels_deltas_old[motor_id] = vels_deltas_new[motor_id];
  motor_pows[motor_id] = vel_adjustment;

  //CAPPING BOUNDS FOR OUTPUT
  if (motor_pows[motor_id] > 100) {
    motor_pows[motor_id] = 100;
  }
  else if (motor_pows[motor_id] < 0) {
    motor_pows[motor_id] = 0;
  }


  //Actually moving the motor to correct direction (or stopping) according to callback input
  if(directions[motor_id]==1){
      motorForward(motor_id,motor_pows[motor_id]);
  }
  else if(directions[motor_id]==-1){
      motorBackward(motor_id,-1*motor_pows[motor_id]);
  }
  else if(directions[motor_id]==0){
      motorStop(motor_id);
  }


}




void MotorPIDs::update_velocities(){
  int repeats=3;
  for(int i=0;i<ROTARY_COUNT;i++){
   velocities[i]=0.0; 
  }
  MotorPIDs::printVelocities();
  for(int i=0;i<repeats;i++){
    Wire.requestFrom(ROTARY_SLAVE_ADDRESS, ROTARY_COUNT);
    for (int i = 0; i < ROTARY_COUNT; i++) {
      velocities[i] += (double) Wire.read();  // Must cast to signed 8-bit type
    }
    
    MotorPIDs::printVelocities();
    delay(50);
  }
  for(int i=0;i<ROTARY_COUNT;i++){
   velocities[i]=(velocities[i]/(double)repeats)*(PI/180)*20; 
  }
  
}

void MotorPIDs::printVelocities() {

  //Serial.print("Motor velocities: ");

  for (int i = 0; i < ROTARY_COUNT; i++) {
    /*
    Serial.print(velocities[i]);
    Serial.print(' ');
    */
  }
  // Serial.println();
  
}


MotorPIDs::MotorPIDs(){
        //TODO: NEED TO SEE IF C INITIALISES TO ZERO AUTOMATICALLY

        // SDPsetup();


       

}
