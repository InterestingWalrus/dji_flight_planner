#ifndef FLIGHT_BASE
#define FLIGHT_BASE

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/FusedGps.h>
#include "dji_sdk/dji_sdk.h"
#include <djiosdk/dji_vehicle.hpp>
#include <dji_sdk/dji_sdk_node.h>


#include "m100_flight_planner/mobile_comm.h"

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


class FlightBase
{
    protected:
        ros::NodeHandle nh;
        ros::ServiceClient ctrl_authority_service;
        ros::ServiceClient drone_task_service;
        ros::ServiceClient query_version_service;
        ros::ServiceClient drone_activation_service;
        ros::ServiceClient set_local_pos_reference;
        ros::ServiceClient mobile_data_service;
        ros::Subscriber gps_subscriber;
        ros::Subscriber gps_health_subscriber;
        ros::Subscriber flight_status_subscriber;
        ros::Subscriber height_subscriber;
        ros::Subscriber gps_fused_subscriber;
        ros::Subscriber attitude_subscriber;
        ros::Subscriber local_position_subscriber;
        ros::Subscriber mobile_data_subscriber;
        ros::Subscriber velocity_subscriber;


        uint8_t flight_status = 255;
        uint8_t display_mode  = 255;




    public:
        FlightBase();
        ~FlightBase();

        //TODO Deprecate 
        void fused_gps_callback(const dji_sdk::FusedGps::ConstPtr& msg);
        void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void flightAnomalyCallback(const dji_sdk::FlightAnomaly::ConstPtr& msg);
        void ekf_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
        void mobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& mobile_data);
        void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
        void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);
        void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);
        void height_callback(const std_msgs::Float32::ConstPtr& msg);
        void MobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& from_mobile_data);
        void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg); 




};


#endif