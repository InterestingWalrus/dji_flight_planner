#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/Activation.h>


#include "m100_flight_planner/mobile_comm.h"

//DJI SDK DRONE LIBRARY
#include <djiosdk/dji_vehicle.hpp>
#include "dji_sdk/dji_sdk.h"


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

#define PI (double) 3.141592653589793
#define C_EARTH (double)6378137.0

static double Deg_To_Rad( double degree)
{
    return degree * (PI/180.0);
};

static double Rad_To_Deg(double rad)
{
  return rad * (180.0/PI);  
};


class FlightControl
{
    public:
        FlightControl();
        void activate(); //
        void takeOff(); //
        void land(); //
        void goHome();
        bool monitoredTakeoff(); 
        bool monitoredLanding();
        bool obtainControl();
        bool releaseControl();
        void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
        void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);
        void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);
        void height_callback(const std_msgs::Float32::ConstPtr& msg);
        bool check_M100();
        bool M100monitoredTakeoff();
        bool M100monitoredLanding();
        bool takeoff_land(int task);
        bool set_local_position();

        float computeTimeToLand(); // based on a 1 m/s descent speed
       




    private:
    ros::NodeHandle nh;
    ros::ServiceClient ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;
    ros::ServiceClient drone_activation_service;
    ros::ServiceClient set_local_pos_reference;
    ros::ServiceClient mobile_data_service;
    ros::Subscriber gps_sub;
    ros::Subscriber gps_health_sub;
    ros::Subscriber flightStatusSub;
    ros::Subscriber height_sub;



    uint8_t flight_status = 255;
    uint8_t display_mode  = 255;
    

    
    sensor_msgs::NavSatFix current_gps; 
    
    uint8_t gps_health;
    float height_above_takeoff;
   














};





#endif // FLIGHT_CONTROL_H