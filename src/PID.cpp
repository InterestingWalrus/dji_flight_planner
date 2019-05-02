// #include "pid.h"
// #include <iostream>

// PID::PID(float kp, float ki, float kd, float max, float min)
// {
//     Kp =  kp;
//     Ki = ki;
//     Kd = kd;
//     Max = max;
//     Min = min;

//     std::cout << "Kp: " << Kp << std::endl; 
//     std::cout << "Ki: " << Ki << std::endl; 
//     std::cout << "Kd: " << Kd << std::endl; 
//     std::cout << "Max: " << Max << std::endl; 
//     std::cout << "Min: " << Min << std::endl; 
    

// }


// void PID::resetController()
// {

//     Kp = 0;
//     Ki = 0;
//     Kd = 0;
//     prev_error = 0;
//     integral_error_sum = 0;

// }


// double PID::update(double target_val, double actual_val , double dt )
// {

//     // Error
//      error =  target_val - actual_val;
//      integral_error_sum += error* dt;  // Integral
//      prev_error = error;


//     // Proportional Term 
//     double P_out = Kp * error;

//     // Integral term 
//     double I_out = Ki * integral_error_sum;

//     // Derivative Term
//     double derivative = (error - prev_error) / dt;
//     double D_out = Kd * derivative;

//     // Sum the parts
//     double output = P_out + I_out + D_out;
    
//     //ROS_INFO("Output: %f " , dt);

//     if(output > Max)
//     {
//         output = Max;
//     }

//     if(output < Min)
//     {
//         output = Min;
//     }

//     return output;

// }
#include <iostream>
#include "m100_flight_planner/PID.h"

void Pid_control::PID_init(float kp, float ki, float kd, float max, float min)
{
    pid.target_position = 0.0;
    pid.current_position = 0.0;
    pid.err = 0.0;
    pid.prev_error = 0.0;
    pid.velocity_output = 0.0;
    pid.integral = 0.0;
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    pid.max_val = max;
    pid.min_val = min;
}

float Pid_control::PID_update(float current, float target, const double dt)
{
    pid.target_position = target;
    pid.current_position = current;
    pid.err = pid.target_position - pid.current_position;
    pid.integral += pid.err * dt;
     //pid.integral += pid.err ;

    double P =  pid.Kp * pid.err;
    double I =  pid.Ki * pid.integral;
    double D =  pid.Kd * (pid.err - pid.prev_error) / dt;

   // pid.velocity_output = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.prev_error);
   pid.velocity_output = P + I + D;
    pid.prev_error = pid.err;

    if (pid.velocity_output > pid.max_val)
    {
          pid.velocity_output = pid.max_val;
    }
      
    if (pid.velocity_output < pid.min_val)
    {
         pid.velocity_output = pid.min_val;
    }
        
    return pid.velocity_output;
}

// float Pid_control::PID_update(float current, float target)
// {
//     pid.target_position = target;
//     pid.current_position = current;
//     pid.err = pid.target_position - pid.current_position;
//     pid.integral += pid.err ; 

//     pid.velocity_output = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.prev_error);
//     pid.prev_error = pid.err;

//     if (pid.velocity_output > pid.max_val)
//     {
//           pid.velocity_output = pid.max_val;
//     }
      
//     if (pid.velocity_output < pid.min_val)
//     {
//          pid.velocity_output = pid.min_val;
//     }
        
//     return pid.velocity_output;
// }

void Pid_control::PID_reset()
{
    pid.integral = 0.0;
    pid.err = 0.0;
    pid.prev_error = 0.0;

    pid.Kp = 0.0;
    pid.Ki = 0.0;
    pid.Kd = 0.0;
    
}