
#include "Arduino.h"
#include "Mind.h"


 void Mind::rot(float theta){
  
  temp_rot[0][0]=cos(theta);
  temp_rot[0][1]=-sin(theta);
  temp_rot[1][0]=sin(theta);
  temp_rot[1][1]=cos(theta);
  
  
}

float Mind::project(mtx_type* b,mtx_type* a){
  
 //proj_a(b) = (a dot b)/|a|
 return (a[0]*b[0]+a[1]*b[1])/(sqrt(a[0]*a[0]+a[1]*a[1]));
}

void Mind::update_motor_powers(){
  // we need to project the heading onto the forward and right vectors
  motor_foward_power = (Mind::project(heading,forward));
  motor_right_power = (Mind::project(heading,right));
  
}


 
void Mind::turn_heading(float theta){

//first load rotation matrix
Mind::rot(theta);
//The apply rotation matrix to temporary heading vector:
Matrix.Multiply((mtx_type*)temp_rot,(mtx_type*)heading,N,N,1,(mtx_type*)temp_heading);
//The copy the to the actual heading
Matrix.Copy((mtx_type*)temp_heading,N,N,(mtx_type*)heading);


//update motor powers:
Mind::update_motor_powers();
}

void Mind::turn_actual(float theta){




//first  create rotation matrix:\

Mind::rot(theta);

//The apply rotation matrix to forward and right, using a temporary variable
Matrix.Multiply((mtx_type*)temp_rot,(mtx_type*)forward,N,N,1,(mtx_type*)temp);

forward[0]=temp[0];
forward[1]=temp[1];



Matrix.Multiply((mtx_type*)temp_rot,(mtx_type*)right,N,N,1,(mtx_type*)temp);


right[0]=temp[0];
right[1]=temp[1];
//Now we need to update the motor powers:
Mind::update_motor_powers();


}

void Mind::Print_shit(){
Serial.println();
for(int i=0;i<40;i++)Serial.print("#");
Serial.println();
Matrix.Print((mtx_type*)forward,N,1,"Direction of forward facing motors:");
Matrix.Print((mtx_type*)right,N,1,"Direction of right facing motors:");
Matrix.Print((mtx_type*)heading,N,1,"Direction of heading");
Matrix.Print((mtx_type*)temp,N,1,"temp vector");
Matrix.Print((mtx_type*)transform,N,2,"Transformation matrix");
Matrix.Print((mtx_type*)temp_rot,N,2,"Temporary rot matrix");

Serial.print("Power to forward facing motors: ");
Serial.println(motor_foward_power);
Serial.print("Power to right facing motors: ");
Serial.println(motor_right_power);
for(int i=0;i<40;i++)Serial.print("#");
Serial.println();

}

Mind::Mind(){
Mind::motor_foward_power=1.0f;
Mind::motor_right_power=0.0f;

//Initialise The actual forward and right direction
Mind::forward[0]=0.0f;
Mind::forward[1]=1.0f; // unit vector in the y direction

Mind::right[0]=1.0f;
Mind::right[1]=0.0f; // unit vector in the x direction

// initialise the heading to the initial forward direction
Mind::heading[0]=0.0f;
Mind::heading[1]=1.0f; 

Mind::temp[0]=0.f;
Mind::temp[1]=0.f;
}


void Mind::turn_left(bool a,bool h){
    //actuall turning code goes here
    //The update the brain
    if(a)Mind::turn_actual(PI/2.0f);
    if(h)Mind::turn_heading(PI/2.0f);
}

void Mind::turn_right(bool a,bool h){
    //actuall turning code goes here
    //The update the brain
    if(a)Mind::turn_actual(-PI/2.0f);
    if(h)Mind::turn_heading(-PI/2.0f);
}
void Mind::reverse(bool a,bool h){
    //actuall turning code goes here
    //The update the brain
    if(a)Mind::turn_actual(PI);
    if(h)Mind::turn_heading(PI);
}

