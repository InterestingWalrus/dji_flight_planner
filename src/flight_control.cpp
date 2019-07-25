#include "m100_flight_planner/flight_control.h"

using namespace DJI::OSDK;

FlightControl::FlightControl()
{
   
 

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


bool FlightControl::takeoffLand(int task)
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

  if(!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
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
  float N3_land = computeTimeToLand();
  
  ros::Time start_time = ros::Time::now();
  
  if(!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_LAND)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: landDrone
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_AUTO_LANDING &&
         ros::Time::now() - start_time < ros::Duration(N3_land)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(N3_land)) {
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
          ros::Time::now() - start_time < ros::Duration(N3_land)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND)
  {
    ROS_ERROR_ONCE("Failed Landing!");
    return false;
  }
  else
  {
    ROS_INFO("A3/N3 Drone has successfully landed");
     start_time = ros::Time::now();
     ros::spinOnce();
    
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
  if(!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
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
      ROS_INFO ("Flight status: Drone Taking off");
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
    float droneLandSpeed;
    float current_height;

    // Use M100 for this
      if(check_M100())
    {
      ROS_INFO("Drone is a M100 Variant");
      
      droneLandSpeed = 1;
      current_height = height_above_takeoff;
      timeToLand = (current_height / droneLandSpeed) ;

      // Maximum time M100 drone will take to land from an altitude of 100 metres is 55 seconds.. 
      // TODO: Test A3/N3 Time. 
      // Tested in simulator
      //TODO use Velocity Z to compute variable time to land)

      if (timeToLand > 55)
      {
        timeToLand = 55;
      }

      ROS_INFO_THROTTLE(5, "Time required to land is %f", timeToLand);
    }

    else
    {
      ROS_INFO_ONCE("Drone is a N3/A3 Type");
    
      droneLandSpeed = fabs(velocity_data.vector.z);
      current_height = height_above_takeoff;

     if(current_height >= 40)
     {
       timeToLand = (current_height / droneLandSpeed) + 50 ;
     }

     else
     {
        timeToLand = (current_height / droneLandSpeed) ;  
        
     }
     
 
      if(std::isnan(timeToLand))
      {
        ROS_INFO_ONCE("IS NAnnanananana");
        timeToLand = 0;
      }

    // empirically time to land from the N3 is 90 seconds.
    // so cap time to land to 90 seconds

     if(timeToLand > 90)
     {
       timeToLand = 90;
     }

      if(!std::isinf(timeToLand))
      {
        ROS_INFO_THROTTLE(5, "Time required to land is %f", timeToLand);
       // ROS_INFO("Speed %f", droneLandSpeed);

      } 
    }
  

   return timeToLand + 5; 
}

bool FlightControl::M100monitoredLanding()
{
  ros::Time start_time = ros::Time::now();

  float rosTime_to_land = computeTimeToLand() + 5;
  float home_altitude = current_gps_location.altitude;
  

  if(!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_LAND))
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
      current_gps_location.altitude - home_altitude > 1.0)
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
