
#include <SDPArduino.h>
#include <Wire.h>
#include<Arduino.h>

#ifndef MotorPIDs_h
#define MotorPIDs_h




class MotorPIDs
{   


    double update_past(int motor_id,double new_error);
    void update_velocities();
    

    public:
    void MotorPIDsinit();
    void setup_PID(int motor_id, double Kp, double Ki, double Kd);
    void setSetpointAndDirection(int motor_id,double vel_desired, int direction);
    void update_motor_PID(int motor_id);
    void printVelocities();
    
        MotorPIDs();
        static const int ROTARY_SLAVE_ADDRESS=5;
        static const int ROTARY_COUNT=6;
        static const int PAST=150;

        //to calculate velocities
        double velocities[ROTARY_COUNT];

        // GLOBAL VALUES for ALL PIDS
        double motor_pows[ROTARY_COUNT];
        double vels_deltas_new[ROTARY_COUNT];
        double vels_deltas_old[ROTARY_COUNT];

        
        //VALUES TO BE CHANGED BY CALLBACK (TODO)
        double Kps[ROTARY_COUNT];
        double Kis[ROTARY_COUNT];
        double Kds[ROTARY_COUNT];
        double vels_desired[ROTARY_COUNT];
        int directions[ROTARY_COUNT];

        //for Ki past entries
        double pasts[ROTARY_COUNT][PAST];
        int past_counters[ROTARY_COUNT];


};
#endif
