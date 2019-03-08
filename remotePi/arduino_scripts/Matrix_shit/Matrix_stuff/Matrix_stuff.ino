#include <MatrixMath.h>


#define N  (2)


float motor_foward_power=0.0f;
float  motor_right_power=0.0f;

mtx_type forward[N];//Actual Forward direction of Robot 
mtx_type right[N]; // Actual Right direction of the robot

mtx_type heading[N]; //The direction the robot wants to move in
mtx_type temp_heading[N]; //temporary vbalue 

mtx_type temp_rot[N][N]; //used to create a new transformation matrix
mtx_type transform[N][N];



void rot(float theta){
  
  temp_rot[0][0]=cos(theta);
  temp_rot[0][1]=-sin(theta);
  temp_rot[1][0]=sin(theta);
  temp_rot[1][1]=cos(theta);
  
}

void turn_heading(float theta){
  
  //first load rotation matrix
  rot(theta);
  //The apply rotation matrix to temporary heading vector:
  Matrix.Multiply((mtx_type*)temp_rot,(mtx_type*)heading,N,N,1,(mtx_type*)temp_heading);
  //The copy the to the actual heading
  Matrix.Copy((mtx_type*)temp_heading,N,N,(mtx_type*)heading);
  
  
  //update motor powers:
  update_motor_powers();
}

float project(mtx_type* b,mtx_type* a){
  
 
 //proj_a(b) = (a dot b)/|a|
 return (a[0]*b[0]+a[1]*b[1])/(sqrt(a[0]*a[0]+a[1]*a[1]));
}

void update_motor_powers(){
  // we need to project the heading onto the forward and right vectors
  motor_foward_power = (project(heading,forward));
  motor_right_power = (project(heading,right));
  
}

void setup(){
  
  Serial.begin(9600);
  
  // Initialise The actual forward and right direction
  forward[0]=0.0f;
  forward[1]=1.0f; // unit vector in the y direction
  
  right[0]=1.0f;
  right[1]=0.0f; // unit vector in the x direction
  
  // initialise the heading to the initial forward direction
  heading[0]=0.0f;
  heading[1]=1.0f; 
  
  Matrix.Print((mtx_type*)heading,N,1,"heading");
  turn_heading(PI/2);
  Matrix.Print((mtx_type*)heading,N,1,"heading");
  Serial.println(motor_foward_power);
  Serial.println(motor_right_power);
  
  
  
}


void loop(){

  
  
}
