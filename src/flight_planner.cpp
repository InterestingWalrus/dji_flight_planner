#include "m100_flight_planner/flight_planner.h"


FlightPlanner::FlightPlanner()
{
    control_pub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

    FlightControl flightControl;

    obtain_control = flightControl.obtainControl();
    bool takeoff_result;

    if(obtain_control == true)
    {
        if(flightControl.set_local_position())
        {
            ROS_INFO("Local Position set successfully! ");
        }
    }

    state = MissionState::IDLE;
    waypoint_finished = false;

    if(flightControl.check_M100())
    {
        ROS_INFO("M100 Drone taking off");
        takeoff_result = flightControl.M100monitoredTakeoff();

    }
    else
    {

        // Drone is an A3/N3 variant
        ROS_INFO("Custom Drone taking off");
        takeoff_result = flightControl.monitoredTakeoff();

    }

    if(takeoff_result)
    {
        reset(current_gps_location, current_local_position);

        ROS_INFO("Initiating mission");

        while(ros::ok())
        {
            ros::spinOnce();
            runMission();
        }

    }

    else
    {
        ROS_INFO("Drone failed to take off");
    }



    
    
}

FlightPlanner::~FlightPlanner()
{
    bool release_control = flightControl.releaseControl();

    if(release_control == true)
    {
        ROS_INFO("Program released control");
    }

    else
    {
        ROS_ERROR("Release Control failed!");
    }
}

void FlightPlanner::step(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti)
{
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

  float speedFactor         = 2;
  float yawThresholdInDeg   = 5;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = Deg_To_Rad(target_yaw);
  double yawThresholdInRad = Deg_To_Rad(yawThresholdInDeg);
  double yawInRad          = toEulerAngle(current_atti).z;
  
  // double yawDesiredRad      = atan2(yOffsetRemaining, xOffsetRemaining);

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
    ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
  }

  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    yCmd = yOffsetRemaining;

    //    if (abs(zOffsetRemaining) >= speedFactor)
    //     yCmd = (zOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
    //   else
    //     zCmd = zOffsetRemaining;

    zCmd = start_local_position.z + target_offset_z;

    /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */
    if (break_counter > 50)
    {
        ROS_INFO("##### Route %d finished....", waypoint_index);
        waypoint_finished = true;
        return;
    }

    else if (break_counter > 0)
    {
        droneControlSignal(0,0,0,0);
        break_counter++;
        return;
    }
    else //break_counter = 0, not in break stage
    {
        // How do you align the yawing??
        // if(std::abs(xOffsetRemaining) < speedFactor || std::abs(yOffsetRemaining < speedFactor) || std::abs(yawDesiredRad - yawInRad)< yawThresholdInRad)
        // {
        //     // don't yaw the drone
        //     droneControlSignal(xCmd, yCmd, zCmd, 0, true, true);
        // }
        droneControlSignal(xCmd, yCmd, zCmd, yawDesiredRad, true, true);

    }

    if (std::abs(xOffsetRemaining) < 0.5 &&std::abs(yOffsetRemaining) < 0.5 && std::abs(zOffsetRemaining) < 0.5
        && std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
        {
            //! 1. We are within bounds; start incrementing our in-bound counter
             inbound_counter ++;
        }

    else
    {
       if (inbound_counter != 0)
        {
            //! 2. Start incrementing an out-of-bounds counter
            outbound_counter ++;
        }
    }

    //! 3. Reset withinBoundsCounter if necessary
    if (outbound_counter > 10)
    {
        ROS_INFO("##### Route %d: out of bounds, reset....", waypoint_index);
        inbound_counter  = 0;
        outbound_counter = 0;
    }

    if (inbound_counter > 50)
    {
        ROS_INFO("##### Route %d start break....", waypoint_index);
        break_counter = 1;
    }

}


void FlightPlanner::prepareFlightPlan()
{
    sensor_msgs::NavSatFix flightWaypoint;
    flightWaypoint.latitude = 
    flightWaypoint.longitude = 
    flightWaypoint.altitude = 


    appendFlightPlan(flightWaypoint);

    
}

 bool FlightPlanner::isMissionFinished()
 {
     // if waypoint count isn't zero and is equal to number of waypoint index, 
     // we can safely assume the mission is finished..

     if(waypoint_count != 0 && waypoint_count == waypoint_index)
     {
        return true;
     }

     else
     {
         return false;
     }
 }

void FlightPlanner::droneControlSignal(double x, double y, double z, double yaw, bool use_yaw_rate, bool use_ground_frame)
{
    sensor_msgs::Joy controlPosYaw;

    controlPosYaw.axes.push_back(x);
    controlPosYaw.axes.push_back(y);
    controlPosYaw.axes.push_back(z);
    controlPosYaw.axes.push_back(yaw);

    if(use_yaw_rate && use_ground_frame) // using yaw rate and ground frame
    {
        controlPosYaw.axes.push_back(control_flag); 
    }

    else if (use_yaw_rate) // using yaw rate and body frame
    {
         controlPosYaw.axes.push_back(control_flag_fru); 
    }

    else if (use_ground_frame) // using yaw angle and ground frame
    {
         controlPosYaw.axes.push_back(control_flag_yaw_angle_enu); 
    }

    else // using yaw angle and UAV Body frame
    {
         controlPosYaw.axes.push_back(control_flag_yaw_angle_fru); 
    }

    control_pub.publish(controlPosYaw);

}

void FlightPlanner::reset(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Point &current_local_pos)
{
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;

    bool waypoint_finished = true;
    state = MissionState::NEW_WAYPOINT;
    setWaypoint(flight_plan[waypoint_index]);
    // increment waypoint to next waypoint
    waypoint_index++;

    ROS_INFO("Set new waypoint %d / %d at :", waypoint_index , waypoint_count );
    start_gps_location = current_gps;
    start_local_position = current_local_pos;


}


void FlightPlanner::setWaypoint(sensor_msgs::NavSatFix newWaypoint)
{
   geometry_msgs::Vector3 offset_from_target;

   // get GPS offset
   localOffsetFromGpsOffset(offset_from_target, newWaypoint, start_gps_location );
   ROS_INFO("new Waypoint target offset x: %f y: %f z: %f ", offset_from_target.x, offset_from_target.y, offset_from_target.z);

   // pass local offsets into global variable
   target_offset_x = offset_from_target.x;
   target_offset_y = offset_from_target.y;
   target_offset_z = offset_from_target.z;

}

bool FlightPlanner::reachedWaypoint()
{
   if(!waypoint_finished)
   {
       return false;
   }

   else
   return true;
}

void FlightPlanner::appendFlightPlan(sensor_msgs::NavSatFix newWaypoint)
{

  // add new waypoint to vector
    flight_plan.push_back(newWaypoint);

    // update number of waypoints
    waypoint_count = flight_plan.size();

    ROS_INFO("Waypoint # %d added", waypoint_count);

}

void FlightPlanner::onWaypointReached()
{
    
    if(waypoint_index == 4)
    {
        bool performTask;

        if(flightControl.check_M100())
        {
             performTask = flightControl.M100monitoredLanding();

        }

        else
        {
            //performTask = flightControl.monitoredLanding();
            // This doesn;t work on S1000 for now.
            flightControl.land();
        }
        

        if(performTask)
        {
            // Pause for a period of time here...
            ros::Duration(30).sleep();

            flightControl.monitoredTakeoff();
        }

        else
        {
            ROS_ERROR("Unsucessful landing of drone");
        }
    }

    state = MissionState::ARRIVED;
}

void FlightPlanner::onMissionFinished()
{
    state = MissionState::FINISHED;
    // clear flight plan
    flight_plan.clear();
    waypoint_count = 0;
    waypoint_index = 0;
}
void FlightPlanner::localOffsetFromGpsOffset(geometry_msgs::Vector3& deltaENU, sensor_msgs::NavSatFix& target, sensor_msgs::NavSatFix& origin)
{
        double deltaLon = target.longitude - origin.longitude;
        double deltaLat = target.latitude - origin.latitude;

        deltaENU.y = Deg_To_Rad(deltaLat) * C_EARTH;
        deltaENU.x = Deg_To_Rad(deltaLon) * C_EARTH * cos(Deg_To_Rad(target.latitude));
        deltaENU.z = target.altitude - origin.altitude;
}

geometry_msgs::Vector3 FlightPlanner::toEulerAngle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 ans;

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    return ans;

}

void FlightPlanner::runMission()
{

    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    /// check what state the mission is in:

    // run in 50Hz loop
    if(elapsed_time > ros::Duration(0.02))
    {
        start_time = ros::Time::now();
        switch(state)
        {
            case MissionState::IDLE:
            {
                if(waypoint_count != 0)
                {
                    // reset mission and set waypoint
                    // NEW WAYPOINT flag is set in reset function
                    reset(current_gps_location, current_local_position);
                    ROS_INFO("MISSION START ROUTE %d / %d ", waypoint_count, waypoint_index);
                }

                else
                {
                    ROS_INFO_THROTTLE(2, "Mission in idle state, waiting for a mission plan");
                }


                break;
            }

            case MissionState::NEW_WAYPOINT:
            {   
                // if if mission isn't finished, keep stepping through mission.
                if(!reachedWaypoint())
                {
                    step(current_gps_location, current_drone_attitude);
                }

                else
                {
                    onWaypointReached();
                }


                break;
            }

            case MissionState::ARRIVED:
            {
                if(!isMissionFinished())
                {

                    // TODO........Add mission tasks here


                    // if mission isn't finished, reset gps and local position to current position and continue mission.
                    reset(current_gps_location, current_local_position);
                    ROS_INFO("MISSION START ROUTE %d / %d ", waypoint_count, waypoint_index);

                }

                else
                {
                    onMissionFinished();
                }

                break;
            }

            case MissionState::FINISHED:
            {

                flightControl.M100monitoredLanding();

                break;

            }
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Flight Control");

     FlightPlanner FlightPlanner();


    ros::spin();

    return 0;
}

void FlightPlanner::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps_location = *msg;
  

  ROS_INFO_ONCE("GPS Location %f , %f , %f",  current_gps_location.latitude,  current_gps_location.longitude, current_gps_location.altitude);

}