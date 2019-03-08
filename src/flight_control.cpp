#include "m100_flight_planner/flight_control.h"

using namespace DJI::OSDK;

FlightControl::FlightControl()
{
   ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
   drone_task_service     = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
   query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
   drone_activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
   set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

   

   gps_sub = nh.subscribe("dji_sdk/gps_position", 10, &FlightControl::gps_callback, this);
  // gps_sub = nh.subscribe("gps/filtered", 10, &FlightControl::gps_callback, this);        // For EKF control tetsing
   gps_health_sub = nh.subscribe("dji_sdk/gps_health", 10, &FlightControl::gps_health_callback, this);
   flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &FlightControl::flight_status_callback, this);
    height_sub = nh.subscribe("/dji_sdk/height_above_takeoff",10, &FlightControl::height_callback, this);

    
}

void FlightControl::flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void FlightControl::display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

void FlightControl::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps.latitude = msg->latitude;
  current_gps.longitude = msg->longitude;
  current_gps.altitude = msg->altitude;
}
 
void FlightControl::gps_health_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  gps_health = msg->data;
  ROS_INFO_ONCE ("GPS Health: %i", gps_health);
}

void FlightControl::activate()
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



// checks if drone is Matrice M100.
bool FlightControl::check_M100()
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

void FlightControl::goHome()
{
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_GOHOME;

    drone_task_service.call(droneTaskControl);
    ROS_INFO("DRONE Going Home!");

    if(!droneTaskControl.response.result)
    {
        ROS_ERROR("Drone failed to go home");
    }
}

void FlightControl::land()
{
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_LAND;

    drone_task_service.call(droneTaskControl);

    if(!droneTaskControl.response.result)
    {
        ROS_ERROR("Drone failed to land");
    }
}

bool FlightControl::obtainControl()
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


void FlightControl::height_callback(const std_msgs::Float32::ConstPtr& msg)
{
    height_above_takeoff = msg->data;

}

bool FlightControl::takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
   
    return false;
  }

  return true;
}

bool FlightControl::releaseControl()
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


void FlightControl::getFusedGps(double& lat, double& lon, double& alt)
{
  DJI::OSDK::Telemetry::GPSFused gpsfused;

  lat = gpsfused.altitude;

}

/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool FlightControl::monitoredTakeoff()
{

      ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();

  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;

}

// FOR DJI A3/N3
// Untested.
bool FlightControl::monitoredLanding() // WOrk on this later......
{
  
  float rosTime_to_land = computeTimeToLand() + 5;
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: landDrone
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_AUTO_LANDING &&
         ros::Time::now() - start_time < ros::Duration(rosTime_to_land)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(rosTime_to_land)) {
    ROS_ERROR("Landing Failed");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Drone Landing ...");
    ros::spinOnce();
  }

  // Final check: Finished Landing
  while ((display_mode == DJISDK::DisplayMode::MODE_AUTO_LANDING) &&
          ros::Time::now() - start_time < ros::Duration(rosTime_to_land)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if (display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful Landing!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Landing finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;

}

/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool FlightControl::M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();
  
  float home_takeoff = height_above_takeoff;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    ROS_INFO("Did this fail?:");
    return false;
  }

  ros::Duration(7).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 15 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(15))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR || height_above_takeoff - home_takeoff < 0.6)
  {
   
  
    if (flight_status == DJISDK::M100FlightStatus::M100_STATUS_FINISHED_LANDING)
    {
      ROS_ERROR ("Flight status: Drone Finished Landing");

    }

    if (flight_status == DJISDK::M100FlightStatus::M100_STATUS_LANDING)
    {
      ROS_ERROR ("Flight status: Drone Landing");

    }

    if (flight_status == DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND)
    {
      ROS_ERROR ("Flight status: Drone still on the ground");

    }

    if (flight_status == DJISDK::M100FlightStatus::M100_STATUS_TAKINGOFF)
    {
      ROS_ERROR ("Flight status: Drone Taking off");

    }

    ROS_INFO("Home Takeoff point: %f", home_takeoff);
    ROS_INFO("Current Height above takeoff position: %f", home_takeoff);
    ROS_INFO ("Difference: %f m",  height_above_takeoff - home_takeoff);

    ROS_ERROR("Takeoff failed.");

    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

// returns the time required for the drone to land. 
float FlightControl::computeTimeToLand()
{

  int droneLandSpeed = 1;



  float current_height = height_above_takeoff;

  float timeToLand = (current_height / droneLandSpeed) ;

  // Maximum time drone will take to land from an altitude of 100 metres is 55 seconds.. 
  // Tested in simulator
  //TODO use Velocity Z to compute variable time to land)

  if (timeToLand > 55)
  {
    timeToLand = 55;
  }

  ROS_INFO("Time required to land is %f", timeToLand);

   return timeToLand; 
}

bool FlightControl::M100monitoredLanding()
{
  ros::Time start_time = ros::Time::now();

  float rosTime_to_land = computeTimeToLand() + 5;
  float home_altitude = current_gps.altitude;
  

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // compute wait time to be based on relative altitude of drone / drone descent speed : which is at 1 m/s
  while (ros::Time::now() - start_time < ros::Duration(rosTime_to_land) || flight_status == DJISDK::M100FlightStatus::M100_STATUS_LANDING)
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND ||
      current_gps.altitude - home_altitude > 1.0)
  {
    ROS_ERROR_ONCE("Landing failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO_ONCE("Successful Landing!");
    ros::spinOnce();
  }

  return true; 
}

bool FlightControl::set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

//TODO TAke off bug for when the drone takes off again after landing at a waypoint.