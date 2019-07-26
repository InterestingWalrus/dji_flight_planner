#include "m100_flight_planner/flight_base.h"

using namespace DJI::OSDK;

FlightBase::FlightBase()
{
   ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
   drone_task_service     = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
   query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
   drone_activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
   set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

   gps_subscriber = nh.subscribe("dji_sdk/gps_position", 10, &FlightBase::gpsCallback, this);
   gps_health_subscriber = nh.subscribe("dji_sdk/gps_health", 10, &FlightBase::gpsHealthCallback, this);
   flight_status_subscriber = nh.subscribe("dji_sdk/flight_status", 10, &FlightBase::flightStatusCallback, this);
   height_subscriber = nh.subscribe("/dji_sdk/height_above_takeoff",10, &FlightBase::heightCallback, this);
   velocity_subscriber = nh.subscribe("/dji_sdk/velocity", 10,  &FlightBase::velocityCallback, this);  
    local_position_subscriber = nh.subscribe("/dji_sdk/local_position", 10, &FlightBase::localPositionCallback, this);
   // local_position_subscriber = nh.subscribe("odometry/filtered", 10, &FlightBase::ekf_odometry_callback, this); // For EKF control testing
    attitude_subscriber = nh.subscribe("/dji_sdk/attitude", 10, &FlightBase::attitudeCallback, this);


   // SET Which GPS topic to subscribe to based on which drone is being used..
   // Currently can't use the same subscriber as /dji_sdk/fused_gps isn't a NAVSATFIX Type 
   //TODO To be deprecated
   if(checkM100)
   {
       ROS_INFO("DJI M100");
        // gps_subscriber = nh.subscribe("/dji_sdk/gps_position", 10, &FlightPlanner::gps_callback, this);
   }

   else
   {
        ROS_INFO("DJI N3/A3");
   }

}

// void FlightBase::ekfGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
// {
//     current_gps_location = *msg;

//     ROS_INFO_ONCE("GPS Location %f , %f , %f",  current_gps_location.latitude,  current_gps_location.longitude, current_gps_location.altitude);
// }

void FlightBase::localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
     ros::Time now = ros::Time::now();
     ros::Duration time = now - prev_time;
     current_local_position = msg->point;
     prev_time = now;

     dt = time.toSec(); 

}

// void FlightBase::ekfOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
// {
//    current_local_position = msg->pose.pose.position;

// }

void FlightBase::attitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    current_drone_attitude = msg->quaternion;
}

void FlightBase::heightCallback(const std_msgs::Float32::ConstPtr& msg)
{
    height_above_takeoff = msg->data;

}

void FlightBase::velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    velocity_data.vector = msg->vector;
    velocity_data.header = msg->header;
}

void FlightBase::flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    flight_status = msg->data;
}

void FlightBase::displayModeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    display_mode = msg->data;
}

void FlightBase::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps_location.latitude = msg->latitude;
  current_gps_location.longitude = msg->longitude;
  current_gps_location.altitude = msg->altitude;
}

void FlightBase::gpsHealthCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  gps_health = msg->data;
  ROS_INFO_ONCE ("GPS Health: %i", gps_health);
}

void FlightBase::activate()
{
   dji_sdk::Activation activation;
   drone_activation_service.call(activation);
   
   if(!activation.response.result)
   {
       ROS_ERROR("Drone control program not activated. Please check your app key");
   }

   else
   {
       ROS_INFO("Activation Successful");
   }

}

bool FlightBase::setLocalPosition()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

// checks if drone is Matrice M100.
bool FlightBase::checkM100()
{

    dji_sdk::QueryDroneVersion query;
    query_version_service.call(query);

    if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
    {
        return true;
    }

    else
    {
        return false;
    }

}

bool FlightBase::releaseControl()
{
   dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 0;
    ctrl_authority_service.call(authority);
    if(authority.response.result)
    {
        ROS_INFO("Program has released control");
        return true;
    }

    if(!authority.response.result)
    {
        ROS_INFO("Program failed to release control");
        return true;
    }
}

bool FlightBase::obtainControl()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    ctrl_authority_service.call(authority);
    if(authority.response.result)
    {
        ROS_INFO("Program has obtained control");
        return true;

    }
    else
    {
        if(authority.response.ack_data == 3 && authority.response.cmd_set == 1 && authority.response.cmd_id == 0)
        {
            ROS_INFO("Call control Authority again");
        }
        else
        {
            ROS_ERROR("Failed to obtain control authority");
            return false;
        }
    }

    return true;

}

geometry_msgs::Vector3 FlightBase::toEulerAngle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 ans;

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    return ans;

}




