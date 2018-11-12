#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cmath>


class PID
{
   public:
   
   PID();
   void resetController();
    double getE_k();
    double PID_controller(double velocity);


   private:
   double Kp, Ki, Kd, e_k, prev_e_k, integral, derivative; 
   

   // return last measured error
  

     



};