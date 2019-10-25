
#ifndef _PID_H_
#define _PID_H_

#include <ros/ros.h>

typedef struct _pid
{
    float current_effort;
    float target_effort;
    float last_position;  // to get rid of derivative kick
    float err;
    float prev_error;
    float Kp, Ki, Kd;
    float velocity_output;
    float integral;
    float max_effort;
    float min_effort;
    double sampleTime;
} Pid;

class Pid_control
{
  public:
    void PIDinit(float kp, float ki, float kd, float max, float min);
    void PIDreset();
    float PIDupdate(float target);
    float clamp(float input, float min_value, float max_value); // returns a value between the min and max boundaries
  
  private:
    int index;
    Pid pid;
};

#endif