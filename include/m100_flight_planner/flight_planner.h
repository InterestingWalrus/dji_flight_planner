#ifndef FLIGHT_PLANNER_H
#define FLIGHT_PLANNER_H

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include "dji_sdk/dji_sdk.h"

#include "m100_flight_planner/flight_state.h"
#include "m100_flight_planner/flight_control.h"

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

enum MissionState
{
    IDLE = 0,
    NEW_WAYPOINT = 1,
    ARRIVED = 2,
    FINISHED = 3    

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
        void step(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti);
        void reset(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Point &current_local_pos);
        bool isMissionFinished();
        bool reachedWaypoint();
        void onWaypointReached();
        void onMissionFinished();
        void runMission();
        void prepareFlightPlan();
        void appendFlightPlan(sensor_msgs::NavSatFix newWaypoint);
        void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaENU, sensor_msgs::NavSatFix& target, sensor_msgs::NavSatFix& origin);
        void droneControlSignal(double x, double y, double z, double yaw, bool use_yaw_rate = true, bool use_ground_frame = true);
        geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);
    
   
    private:

    int state;
    int waypoint_index;
    int waypoint_count;
    int outbound_counter;
    int inbound_counter;
    int break_counter;

    float target_offset_x;
    float target_offset_y;
    float target_offset_z;
    float target_yaw;
    sensor_msgs::NavSatFix start_gps_location;
    geometry_msgs::Point start_local_position;

    sensor_msgs::NavSatFix current_gps_location;
    geometry_msgs::Point current_local_position;
    geometry_msgs::Quaternion current_drone_attitude;

    std::vector<sensor_msgs::NavSatFix> flight_plan;

    bool waypoint_finished;
    bool obtain_control;

    // Use this control flag (x-y ground frame is horizontal frame in ENU)
    uint8_t control_flag = (DJISDK::VERTICAL_VELOCITY| DJISDK::HORIZONTAL_VELOCITY|DJISDK::YAW_RATE |DJISDK::HORIZONTAL_GROUND |DJISDK::STABLE_ENABLE); 

    // control flag (x-y body frame in horizontal FRU frame )
    uint8_t control_flag_fru = (DJISDK::VERTICAL_VELOCITY| DJISDK::HORIZONTAL_VELOCITY|DJISDK::YAW_RATE |DJISDK::HORIZONTAL_BODY |DJISDK::STABLE_ENABLE); 

   // control flag (x-y body frame in horizontal FRU frame using yaw angle )
    uint8_t control_flag_yaw_angle_fru = (DJISDK::VERTICAL_VELOCITY| DJISDK::HORIZONTAL_VELOCITY|DJISDK::YAW_ANGLE |DJISDK::HORIZONTAL_BODY |DJISDK::STABLE_ENABLE);
    // control flag (x-y body frame in horizontal ENU frame using yaw angle )
    uint8_t control_flag_yaw_angle_enu = (DJISDK::VERTICAL_VELOCITY| DJISDK::HORIZONTAL_VELOCITY|DJISDK::YAW_ANGLE |DJISDK::HORIZONTAL_GROUND |DJISDK::STABLE_ENABLE);


    DroneState droneState;
    FlightControl flightControl;
    ros::NodeHandle nh;
    ros::Publisher control_pub;
    





    

};





#endif // FLIGHT_PLANNER_H