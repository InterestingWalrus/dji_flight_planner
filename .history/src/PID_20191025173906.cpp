#include <iostream>
#include "m100_flight_planner/PID.h"

void Pid_control::PIDinit(float kp, float ki, float kd, float max, float min)
{
    pid.current_effort = 0.0;
    pid.target_effort = 0.0;
    pid.err = 0.0;
    pid.last_position = 0.0;
    pid.prev_error = 0.0;
    pid.velocity_output = 0.0;
    pid.integral = 0.0;
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    pid.max_effort = max;
    pid.min_effort = min;
    pid.sampleTime = 0.02;

    // Anti windup
    // Probably dont need to call this in the constructor
    if (pid.integral > pid.max_effort)
    {
        pid.integral = pid.min_effort;
    }

    else if (pid.integral < pid.min_effort)
    {
        pid.integral = pid.min_effort;
    }
}

float Pid_control::clamp(float input, float min_value, float max_value)
{
    return std::min(std::max(input, min_value), max_value);
}

void Pid_control::PIDreset()
{
    pid.integral = 0.0;
    pid.err = 0.0;
    pid.prev_error = 0.0;

    pid.Kp = 0.0;
    pid.Ki = 0.0;
    pid.Kd = 0.0;
}

float Pid_control::PIDupdate(float target)
{
    pid.target_effort = target;
    pid.current_effort = pid.current_effort;

    pid.err = pid.target_effort - pid.current_effort;
    pid.integral += pid.err * pid.sampleTime;

    if (pid.integral > pid.max_effort)
    {
        pid.integral = pid.max_effort;
    }

    else if (pid.integral < pid.min_effort)
    {
        pid.integral = pid.min_effort;
    }

    double delta_input = pid.current_effort - pid.last_position;

    double P = pid.Kp * pid.err;
    double I = pid.Ki * pid.integral;
    //double D =  pid.Kd * (pid.err - pid.prev_error) /  pid.sampleTime;
    double D = pid.Kd * (delta_input) / pid.sampleTime; // Derivative Kick elimination

    pid.velocity_output = P + I + D;

    // Update previous positions.
    pid.prev_error = pid.err;
    pid.last_position = pid.current_effort;

    if (pid.velocity_output > pid.max_effort)
    {
        pid.velocity_output = pid.max_effort;
    }

    else if (pid.velocity_output < pid.min_effort)
    {
        pid.velocity_output = pid.min_effort;
    }

    return pid.velocity_output;
}