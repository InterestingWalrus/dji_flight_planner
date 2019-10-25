#ifndef FLIGHT_PLANNER_H
#define FLIGHT_PLANNER_H

#include "m100_flight_planner/PID.h"
#include "m100_flight_planner/flight_base.h"
#include "m100_flight_planner/flight_control.h"

#include <queue>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tuple>

 class FlightControl;

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


class FlightPlanner : public FlightBase
{
    public:
        FlightPlanner();
        ~FlightPlanner();
        void setWaypoint(sensor_msgs::NavSatFix newWaypoint); 
        void step();
        void stepHome();
        void stepYaw();
        void getLocalPositionOffset(geometry_msgs::Vector3&  deltaENU, sensor_msgs::NavSatFix& target, sensor_msgs::NavSatFix& origin);
        void reset();
        bool isMissionFinished();
        bool reachedWaypoint();
        void onWaypointReached();
        void onMissionFinished();
        void runMission();
        void returnHome();
        void prepareFlightPlan(double lat, double lon, double alt, unsigned char sampling_task, float samplingTime);
        void appendFlightPlan(sensor_msgs::NavSatFix newWaypoint, unsigned char land, float samplingTime);
        void droneControlSignal(double x, double y, double z, double yaw, bool use_yaw_rate = true, bool use_ground_frame = true);     
        void keyboardControl();
        void setZOffset(double offset);

        Eigen::Vector3d getEffort(Eigen::Vector3d& target);
        Eigen::Vector3d getHomeEffort(Eigen::Vector3d& target);
        Eigen::Vector3d setTarget(float x, float y, float z);
        Eigen::Vector3d setHomeTarget(float x, float y, float z);
        float setYaw(float yaw);

        void flightAnomalyCallback(const dji_sdk::FlightAnomaly::ConstPtr& msg);
        void mobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& mobile_data);


 
   
    private:

        MobileComm mobileCommManager;    
        FlightControl flightControl;

        int state; // drone state
        int waypoint_index;
        int waypoint_count;

        Eigen::Vector3d target_position_vector;  // position offsets between waypoints
        Eigen::Vector3d home_position_vector;  // calculate position offsets between current position and home point.
        float target_yaw_angle;
        float desired_yaw_angle;
        float current_yaw_angle;
        double z_offset_takeoff;
        float yaw_limit;
        float distance_to_setpoint;
        float home_distance;
        bool drone_version;  // return drone version.

        // get data from the android device 
        dji_sdk::MobileData data_from_mobile;
        unsigned char data_to_mobile[10];
        
        // drone takeoff absolute and local positions
        sensor_msgs::NavSatFix start_gps_location;
        geometry_msgs::Point start_local_position;
        sensor_msgs::NavSatFix home_start_gps_location;
        sensor_msgs::NavSatFix home_gps_location;
       

       // flight plan to store waypoints for missions
        std::vector<sensor_msgs::NavSatFix> flight_plan;

       // check if landing is required at the waypoint
        //std::queue< std::pair < std::vector<sensor_msgs::NavSatFix> , unsigned char > > waypoint_lists;

        std::queue< std::tuple < std::vector<sensor_msgs::NavSatFix> , unsigned char, float > > waypoint_lists;

        bool waypoint_finished;
        bool yaw_flag;
        bool takeoff_result;
        bool home_reached;

        // convert data from the mobile device to waypoint and mission parameters
        unsigned char latitude_array[8] = {0};
        unsigned char longitude_array[8] = {0};
        unsigned char altitude_array[4] = {0};
        unsigned char speed_array[4] = {0};
        unsigned char sampling_time_array[4] = {0};
        unsigned char missionEnd = 0;
        unsigned char task;


        // for parsing the data from the mobile interface.
        bool hover_flag;
        double latitude;
        double longitude;
        float altitude;
        float speedFactor;
        float samplingTime;
        int checkMissionEnd;
        std::string command;
    


        // Use this control flag (x-y ground frame is horizontal frame in ENU)
        uint8_t control_flag = (DJISDK::VERTICAL_VELOCITY| 
                                DJISDK::HORIZONTAL_VELOCITY|
                                DJISDK::YAW_RATE|
                                DJISDK::HORIZONTAL_GROUND|
                                DJISDK::STABLE_ENABLE); 

        // control flag (x-y body frame in horizontal FRU frame )
        uint8_t control_flag_fru = (DJISDK::VERTICAL_VELOCITY| 
                                    DJISDK::HORIZONTAL_VELOCITY|
                                    DJISDK::YAW_RATE|
                                    DJISDK::HORIZONTAL_BODY|
                                    DJISDK::STABLE_ENABLE); 

        // control flag (x-y body frame in horizontal FRU frame using yaw angle )
        uint8_t control_flag_yaw_angle_fru = (DJISDK::VERTICAL_VELOCITY| 
                                              DJISDK::HORIZONTAL_VELOCITY|
                                              DJISDK::YAW_ANGLE|
                                              DJISDK::HORIZONTAL_BODY|
                                              DJISDK::STABLE_ENABLE);

        // control flag (x-y body frame in horizontal ENU frame using yaw angle )
        uint8_t control_flag_yaw_angle_enu = (DJISDK::VERTICAL_VELOCITY|
                                              DJISDK::HORIZONTAL_VELOCITY|
                                              DJISDK::YAW_ANGLE| 
                                              DJISDK::HORIZONTAL_GROUND| 
                                              DJISDK::STABLE_ENABLE);  
        
        ros::Publisher control_publisher;
        
     

};





#endif // FLIGHT_PLANNER_H