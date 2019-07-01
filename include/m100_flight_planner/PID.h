
#ifndef _PID_H_
#define _PID_H_

#include <ros/ros.h>


typedef struct _pid
{
    float target_position;
    float current_position;
    float last_position;  // to get rid of derivative kick
    float err;
    float prev_error;
    float Kp, Ki, Kd;
    float velocity_output;
    float integral;
    float max_val;
    float min_val;
    double sampleTime;
} Pid;

class Pid_control
{
  public:
    void PID_init(float kp, float ki, float kd, float max, float min);
    float PID_update(float current, float target, const double dt);
   //float PID_update(float current, float target);
    void PID_reset();

  private:
    int index;
    Pid pid;
    ros::Time lastMessageTime;
   
};


#endif