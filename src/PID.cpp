#include <iostream>
#include "m100_flight_planner/PID.h"

void Pid_control::PID_init(float kp, float ki, float kd, float max, float min)
{
    pid.target_position = 0.0;
    pid.current_position = 0.0;
    pid.err = 0.0;
    pid.last_position = 0.0;
    pid.prev_error = 0.0;
    pid.velocity_output = 0.0;
    pid.integral = 0.0;
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    pid.max_val = max;
    pid.min_val = min;
    pid.sampleTime = 0.02;

    // Anti windup  
    // Probably dont need to call this in the constructor
    if (pid.integral > pid.max_val)
    {
        pid.integral = pid.max_val;
    }

    else if (pid.integral < pid.min_val)
    {
        pid.integral = pid.min_val;
    }
}

float Pid_control::PID_update(float current, float target, const double dt)
{
    
     ros::Time now = ros::Time::now();
     ros::Duration time = now - lastMessageTime;

     if(time.toSec() >= pid.sampleTime)
     {
        pid.target_position = target;
        pid.current_position = current;
        
        pid.err = pid.target_position - pid.current_position;
        pid.integral += pid.err * pid.sampleTime;

        if (pid.integral > pid.max_val)
        {
            pid.integral = pid.max_val;
        }

        else if (pid.integral < pid.min_val)
        {
            pid.integral = pid.min_val;
        }

        double delta_input = pid.current_position - pid.last_position;

        double P =  pid.Kp * pid.err;
        double I =  pid.Ki * pid.integral;
        //double D =  pid.Kd * (pid.err - pid.prev_error) /  pid.sampleTime;
        double D = pid.Kd * (delta_input)/ pid.sampleTime;  // Derivative Kick elimination
   
        pid.velocity_output = P + I + D;

        // Update previous positions. 
        pid.prev_error = pid.err;
        lastMessageTime = now;
        pid.last_position = pid.current_position;

        if (pid.velocity_output > pid.max_val)
        {
            pid.velocity_output = pid.max_val;
        }
            
        else if (pid.velocity_output < pid.min_val)
        {
            pid.velocity_output = pid.min_val;
        }
                
        return pid.velocity_output;
     }

}
            

void Pid_control::PID_reset()
{
    pid.integral = 0.0;
    pid.err = 0.0;
    pid.prev_error = 0.0;

    pid.Kp = 0.0;
    pid.Ki = 0.0;
    pid.Kd = 0.0;
    
}