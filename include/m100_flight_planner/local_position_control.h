#ifndef LOCAL_POS_CONTROL_H
#define LOCAL_POS_CONTROL_H

#include <dji_sdk/SetLocalPosRef.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>

//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>


bool set_local_position();




class LocalFlightPlanner
{

    bool set_local_position();
    void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);
    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
    void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);
    void setTarget(float x, float y, float z, float yaw);
    void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd);



    float target_offset_x;
    float target_offset_y;
    float target_offset_z;
    float target_yaw;
    int target_set_state = 0;

    ros::ServiceClient set_local_pos_reference;
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;

    uint8_t flight_status = 255;
    uint8_t display_mode  = 255;
    uint8_t current_gps_health = 0;
    int num_targets = 0;
    geometry_msgs::PointStamped local_position;
    sensor_msgs::NavSatFix current_gps_position;
  


};


















#endif