#include <Mind.h>

#include <MatrixMath.h>
#include <Wire.h>
#include <SDPArduino.h>


Mind* m = new Mind();


void setup(){
  Serial.begin(9600);
 
  
  
}


void loop(){
  m->Print_shit();
  
  m->turn_right(true,false);
  m->Print_shit();
  
  m->turn_left(false,true);
  //drive
  m->Print_shit();
  
  


  
  
  while(1);
}
