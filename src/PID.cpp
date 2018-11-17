#include "m100_flight_planner/PID.h"

PID::PID()
{
    dt = 0.02;
    Kp= 0.5;
    Ki = 0.001;
    Kd = 0.00001;
    prev_error = 0;
    integral = 0;

}


void PID::resetController()
{

}


double PID::calculate(double target_val, double actual_val , double minVal, double maxVal )
{

    double error = actual_val - target_val;

    // Proportional Term 
    double P_out = Kp * error;

    // Integral term 
    integral += error * dt;
    double I_out = Ki * integral;

    // Derivative Term
    double derivative = (error - prev_error) / dt;
    double D_out = Kd * derivative;

    // Sum the parts
    double output = P_out + I_out + D_out;

    // Clamp by restricting to minimum and maximum values
    if(output > maxVal)
    {
        output = maxVal;
    } 

    else if(output < minVal)
    {
        output = minVal;
    }

    prev_error = error;
    
    //ROS_INFO("Output: %f " , output);

    return output;






}

void PID::setMax(double maxValue)
{
   max = maxValue;
}

void PID::setMin(double minValue)
{

    min = minValue;

}
