#ifndef FLIGHT_BASE
#define FLIGHT_BASE

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
//#include <dji_sdk/FusedGps.h>
#include "dji_sdk/dji_sdk.h"
#include <djiosdk/dji_vehicle.hpp>
#include <dji_sdk/dji_sdk_node.h>

#include "m100_flight_planner/mobile_comm.h"

#define PI (double)3.141592653589793
#define C_EARTH (double)6378137.0

static double DegToRad(double degree)
{
    return degree * (PI / 180.0);
};

static double RadToDeg(double rad)
{
    return rad * (180.0 / PI);
};

//TODO Integrate FlightController Class in OSDK3.9
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

   // ros::Publisher angle_publisher;

    geometry_msgs::Vector3Stamped velocity_data;
    geometry_msgs::Point current_local_position;
    sensor_msgs::NavSatFix current_gps_location;
    geometry_msgs::Quaternion current_drone_attitude;
    //DJI::OSDK::FlightController flightController;
    uint8_t gps_health;
    float height_above_takeoff;
    uint8_t flight_status = 255;
    uint8_t display_mode = 255;
    ros::Time current_time, prev_time;
    double dt;

public:
    FlightBase();
    ~FlightBase();

    void activate(); // Activate the drone for SDK control
    bool checkM100();
    bool obtainControl();
    bool releaseControl();

    //TODO Deprecate
    //void fusedGpsCallback(const dji_sdk::FusedGps::ConstPtr& msg);
    void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
    //void ekfOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    //void ekfGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void attitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);
    void flightStatusCallback(const std_msgs::UInt8::ConstPtr &msg);
    void displayModeCallback(const std_msgs::UInt8::ConstPtr &msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void gpsHealthCallback(const std_msgs::UInt8::ConstPtr &msg);
    void heightCallback(const std_msgs::Float32::ConstPtr &msg);
    void velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
    bool setLocalPosition();
    geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);
};

#endif