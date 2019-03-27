
#include <SDPArduino.h>
#include <Wire.h>
#include<Arduino.h>

#ifndef MotorPIDs_h
#define MotorPIDs_h




class MotorPIDs
{   


    float update_past(int motor_id,float new_error);

    

    public:
    void MotorPIDsinit();
    void setup_PID(int motor_id, float Kp, float Ki, float Kd);
    void setSetpoint(int motor_id,float vel_desired);
    void update_motor_PID(int motor_id);
    void update_velocities();
    void printVelocities();
    void motorMove(int motor_id,int power);
    
        MotorPIDs();
        static const int ROTARY_SLAVE_ADDRESS=5;
        
        static const int PAST=30;

        //to calculate velocities
        static const int ROTARY_COUNT=6;
        float velocities[ROTARY_COUNT];

        // GLOBAL VALUES for ALL PIDS
        float motor_pows[ROTARY_COUNT];
        float vels_deltas_new[ROTARY_COUNT];
        float vels_deltas_old[ROTARY_COUNT];

        
        //VALUES TO BE CHANGED BY CALLBACK (TODO)
        float Kps[ROTARY_COUNT];
        float Kis[ROTARY_COUNT];
        float Kds[ROTARY_COUNT];
        float vels_desired[ROTARY_COUNT];

        //for Ki past entries
        float pasts[ROTARY_COUNT][PAST];
        int past_counters[ROTARY_COUNT];


};
#endif
