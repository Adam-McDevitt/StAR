#ifndef Mind_h

#include<MatrixMath.h>
#include <Arduino.h>
#include <SDPArduino.h>

class Mind
{
    public:
        
        static const int N=2;
        float motor_foward_power;
        float  motor_right_power;

        mtx_type forward[N];//Actual Forward direction of Robot 
        mtx_type right[N]; // Actual Right direction of the robot

        mtx_type temp[N]; //Temporary vector used when rotating the bot

        mtx_type heading[N]; //The direction the robot wants to move in
        mtx_type temp_heading[N]; //temporary vbalue 

        mtx_type temp_rot[N][N]; //used to create a new transformation matrix
        mtx_type transform[N][N];

        void rot(float theta);
        float project(mtx_type* b,mtx_type* a);
        void update_motor_powers();
        void turn_heading(float theta);
        void turn_actual(float theta);
        

    
        Mind();
        void turn_left(bool a,bool h);
        void turn_right(bool a,bool h);
        void reverse(bool a,bool h);

        void Print_shit();
};

#endif