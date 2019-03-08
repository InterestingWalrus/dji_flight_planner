#ifndef FLIGHT_PLANNER_H
#define FLIGHT_PLANNER_H

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include "dji_sdk/dji_sdk.h"
#include <djiosdk/dji_vehicle.hpp>
#include <dji_sdk/dji_sdk_node.h>

#include "m100_flight_planner/flight_control.h"
#include "m100_flight_planner/mobile_comm.h"
#include "m100_flight_planner/PID.h"
#include <queue>


#define PI (double) 3.141592653589793
#define C_EARTH (double)6378137.0

enum MissionState
{
    IDLE = 0,
    NEW_WAYPOINT = 1,
    ARRIVED = 2,
    FINISHED = 3,
    GO_HOME = 4    

};


enum WaypointTask
{
   LAND = 0
};


class FlightPlanner
{
    public:
        FlightPlanner();
        ~FlightPlanner();


        void setWaypoint(sensor_msgs::NavSatFix newWaypoint); 

        // current_atti is used to set yaw_angle for drone. Currently not needed. 
        void step(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti);

        void reset(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Point &current_local_pos);

        void stepHome(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti);
        bool isMissionFinished();
        bool reachedWaypoint();
        void onWaypointReached();
        void onMissionFinished();
        void runMission();

        void returnHome();
        void prepareFlightPlan(double lat, double lon, double alt, unsigned char samplingTask);
        void appendFlightPlan(sensor_msgs::NavSatFix newWaypoint, unsigned char land);
        void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaENU, sensor_msgs::NavSatFix& target, sensor_msgs::NavSatFix& origin);
        void droneControlSignal(double x, double y, double z, double yaw, bool use_yaw_rate = true, bool use_ground_frame = true);
        void droneControlSignalPID(double x, double y, double z, double yaw, bool use_yaw_rate = true, bool use_ground_frame = true);
        geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);


        void MobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& from_mobile_data);
        void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

        void ekf_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void flightAnomalyCallback(const dji_sdk::FlightAnomaly::ConstPtr& msg);

        void ekf_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
        void mobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& mobile_data);


        void keyboardControl();


       


    
   
    private:

    int state; // drone state
    int waypoint_index;
    int waypoint_count;

    // counters for mission and home step functions
    int outbound_counter;
    int inbound_counter;
    int break_counter;

    int home_inbound_counter ;
    int home_outbound_counter;
    int home_break_counter ;
   
    // get data from the android device 
    dji_sdk::MobileData data_from_mobile;
    unsigned char data_to_mobile[10];


     // calculate position offsets between waypoints
    float target_offset_x;
    float target_offset_y;
    float target_offset_z;
    float target_yaw;

    // calculate position offsets between current position and home point.
    float home_target_offset_x;
    float home_target_offset_y;
    float home_target_offset_z;


   // drone takeoff absolute and local positions
    sensor_msgs::NavSatFix start_gps_location;
    geometry_msgs::Point start_local_position;


    sensor_msgs::NavSatFix current_gps_location;
    sensor_msgs::NavSatFix home_start_gps_location;

    sensor_msgs::NavSatFix home_gps_location;
    geometry_msgs::Point current_local_position;
    geometry_msgs::Quaternion current_drone_attitude;

   // flight plan to store waypoints for missions
    std::vector<sensor_msgs::NavSatFix> flight_plan;

   // check if landing is required at the waypoint
    std::queue< std::pair < std::vector<sensor_msgs::NavSatFix> , unsigned char > > waypoint_lists;

    bool waypoint_finished;
    bool obtain_control;
    bool takeoff_result;
    bool homeReached;

    // convert data from the mobile device to waypoint and mission parameters
    unsigned char latitude_array[8] = {0};
    unsigned char longitude_array[8] = {0};
    unsigned char altitude_array[4] = {0};
    unsigned char speed_array[4] = {0};
    unsigned char missionEnd = 0;
    unsigned char task;


    int hover_flag;
    

    double latitude;
    double longitude;
    float altitude;
    float speedFactor;

    int checkMissionEnd;

     std::string command;
   


    // Use this control flag (x-y ground frame is horizontal frame in ENU)
    uint8_t control_flag = (DJISDK::VERTICAL_VELOCITY| DJISDK::HORIZONTAL_VELOCITY|DJISDK::YAW_RATE |DJISDK::HORIZONTAL_GROUND |DJISDK::STABLE_ENABLE); 

    // control flag (x-y body frame in horizontal FRU frame )
    uint8_t control_flag_fru = (DJISDK::VERTICAL_VELOCITY| DJISDK::HORIZONTAL_VELOCITY|DJISDK::YAW_RATE |DJISDK::HORIZONTAL_BODY |DJISDK::STABLE_ENABLE); 

   // control flag (x-y body frame in horizontal FRU frame using yaw angle )
    uint8_t control_flag_yaw_angle_fru = (DJISDK::VERTICAL_VELOCITY| DJISDK::HORIZONTAL_VELOCITY|DJISDK::YAW_ANGLE |DJISDK::HORIZONTAL_BODY |DJISDK::STABLE_ENABLE);
    // control flag (x-y body frame in horizontal ENU frame using yaw angle )
    uint8_t control_flag_yaw_angle_enu = (DJISDK::VERTICAL_VELOCITY| DJISDK::HORIZONTAL_VELOCITY|DJISDK::YAW_ANGLE |DJISDK::HORIZONTAL_GROUND |DJISDK::STABLE_ENABLE);


 
    MobileComm mobileCommManager;
    FlightControl flightControl;
    PID pid;  
    ros::NodeHandle nh;
    ros::Publisher control_pub;

    ros::Subscriber attitude_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber local_position_sub;
    ros::Subscriber mobile_data_subscriber;
    

};





#endif // FLIGHT_PLANNER_H