#ifndef _PID_H_
#define _PID_H_
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cmath>
#include <ros/ros.h>

class PID
{
   public:
   PID();
   void resetController();
   void setMax(double maxValue);
   void setMin(double minValue);
   double calculate(double setpoint, double process_value, double minVal, double maxVal);  // setpoint is what we ant value to be. // process_value is the actual value;

   // error is process value - setpoint.


   private:
   double dt;
   double max;
   double min;
   double Kp;
   double Ki;
   double Kd;
   double prev_error;
   double integral;

  
};


#endif