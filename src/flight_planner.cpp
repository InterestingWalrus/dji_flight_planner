#include "m100_flight_planner/flight_planner.h"

FlightPlanner::FlightPlanner()
{
    control_pub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
    gps_sub = nh.subscribe("/dji_sdk/gps_position", 10, &FlightPlanner::gps_callback, this);
    //gps_sub = nh.subscribe("/gps/filtered", 10, &FlightPlanner::ekf_gps_callback, this);  // for EKF control testing
    local_position_sub = nh.subscribe("/dji_sdk/local_position", 10, &FlightPlanner::local_position_callback, this);
   // local_position_sub = nh.subscribe("odometry/filtered", 10, &FlightPlanner::ekf_odometry_callback, this); // For EKF control testing
    attitude_sub = nh.subscribe("/dji_sdk/attitude", 10, &FlightPlanner::attitude_callback, this);
    mobile_data_subscriber = nh.subscribe<dji_sdk::MobileData>("dji_sdk/from_mobile_data", 10, &FlightPlanner::mobileDataSubscriberCallback, this);

    FlightControl flightControl;
    PID pid;

    ROS_INFO("In control loop");

    obtain_control = flightControl.obtainControl();
   
    if(obtain_control == true)
    {
        if(flightControl.set_local_position())
        {
            ROS_INFO("Local Position set successfully! ");
        }
    }

   // set initial parameters. 

    state = MissionState::IDLE;
    waypoint_finished = false;
    homeReached = false;
    waypoint_index = 0;
    waypoint_count = 0;
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;
    target_yaw = 0;
  
    home_inbound_counter = 0;
    home_outbound_counter = 0;
    home_break_counter = 0;

    data_to_mobile[10] = {0};
    latitude_array[8] = {0};
    longitude_array[8] = {0};
    altitude_array[4] = {0};
    task = 0;
    speed_array[4] = {0};
    missionEnd = 0; 
    
}

 void FlightPlanner::returnHome()
 {
   
    ROS_INFO("Returning home");

     geometry_msgs::Vector3 offset_from_home;

    ROS_INFO(" HOME Waypoint COORDINATES :  %f ,   %f", home_gps_location.latitude, home_gps_location.longitude );

    localOffsetFromGpsOffset(offset_from_home, home_gps_location, current_gps_location );
    ROS_INFO("HOME Waypoint target offset  x: %f y: %f z: %f ", offset_from_home.x, offset_from_home.y, offset_from_home.z);

    // pass local offsets into global variable
    home_target_offset_x = offset_from_home.x;
    home_target_offset_y = offset_from_home.y;
    home_target_offset_z = offset_from_home.z;

    if(!homeReached)
    {
        ROS_INFO_ONCE("On the way back home");
        stepHome(current_gps_location, current_drone_attitude);

    }  
}

void FlightPlanner::mobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& mobile_data)
{
    data_from_mobile = *mobile_data;
    unsigned char data;
    memcpy(&data, &data_from_mobile.data[0], 10);

    unsigned char flight_data[28] = {0};

    unsigned char CMD = data_from_mobile.data[0];


    switch(CMD)
    {
       case 0x2f:
       {
           ROS_INFO("Waypoints received");

           for(int i = 0; i < sizeof(latitude_array); i ++)
            {
                 latitude_array [i] = data_from_mobile.data[i + 1];  
          
            }

            for(int i = 0; i < sizeof(longitude_array); i ++)
            {
                longitude_array [i] = data_from_mobile.data[i + 9] ;
            }   

            for(int i = 0; i < sizeof(altitude_array); i ++)
            {
                altitude_array [i] = data_from_mobile.data[i + 17] ;
            }

             task = data_from_mobile.data[21];
   


            std::reverse(std::begin(latitude_array), std::end(latitude_array));
            std::reverse(std::begin(longitude_array), std::end(longitude_array));
            std::reverse(std::begin(altitude_array), std::end(altitude_array));

            std::memcpy(&latitude, latitude_array, sizeof (double));
            std::memcpy(&longitude, longitude_array, sizeof (double));
            std::memcpy(&altitude, altitude_array, sizeof(float));


            std::cout<< "Landing at Waypoint:" << static_cast<unsigned>(task) << std::endl;
            // Add waypoint to flight plan
            prepareFlightPlan(latitude, longitude, altitude, task);

           break;

       }

       case 0x4d:
       {
           ROS_INFO("Flight Parameters received");

            for(int i = 0; i < sizeof(speed_array); i++)
            {
               speed_array[i] = data_from_mobile.data[i + 1];
            }
                    
            missionEnd = data_from_mobile.data[5];
 
            std::reverse(std::begin(speed_array), std::end(speed_array));
           
            std::memcpy(&speedFactor, speed_array, sizeof(float));
            
             switch(missionEnd)
            {
                case 1:  // NO ACTION
                checkMissionEnd = 1;
                std::cout << "HOVERING ACTION" << std::endl;
                break;

                case 2: //Return Home
                checkMissionEnd = 3;
                 std::cout << "RETURN HOME" << std::endl;

                break;

                case 3:  // Monitored Landing
                 checkMissionEnd = 2;
                 std::cout << "AUTO LAND" << std::endl;
                break;
                
                default:
                break;
            }

            std::cout << std::fixed << speedFactor << "m/s" << std::endl;
            break;
       }

       case 0x3F:
       {
             
               flight_plan.clear();
               waypoint_count = 0;
               waypoint_index = 0;

                state = MissionState::IDLE;

               ROS_INFO("Waypoints cleared, Aircraft now in IDLE State");

           break;        
       }

       case 0x03:
       {
           ROS_INFO("Received command to Land");
           flightControl.land();
           break;
       }

       case 0x01:
       {
           ROS_INFO("Received command for takeoff");
           flightControl.M100monitoredTakeoff();
           break;
       }  

       case 0x02:
       {

           ROS_INFO("Aborting mission: Going home");
           returnHome();
           break;
       }

       case 0x1A:
       {
            ros::Rate loop_rate(50);
            // SET HOME GPS LOCATION here.
            home_gps_location = current_gps_location;      

            ROS_INFO("Logged home GPS location at Lat:%f  , Lon:%f  , Alt:%f ", home_gps_location.latitude, home_gps_location.longitude, home_start_gps_location.altitude);
          
             
            // Set Home GPS Altitude as a 5 metre offset from the current take off altitude of the drone
            home_gps_location.altitude = current_gps_location.altitude +  5.0;
            ROS_INFO("Logged home return GPS Altitude :%f ", home_start_gps_location.altitude);

            if(flightControl.check_M100())
                {
                    ROS_INFO("M100 Drone taking off");
                    takeoff_result = flightControl.M100monitoredTakeoff();        
                }
                else
                {

                    // iF Drone is an A3/N3 variant
                    ROS_INFO("Custom Drone taking off");
                    takeoff_result = flightControl.monitoredTakeoff();
                }

                if(takeoff_result)
                {

                    // set first gps position
                    start_gps_location = current_gps_location;
                    start_local_position = current_local_position;
                                

                    ROS_INFO("Initiating mission");

                    while(ros::ok())
                    {
                        ros::spinOnce();

                        // start mission here
                        runMission();

                        loop_rate.sleep();
                    }


                }
        }

            default:
            {
                    break;
            }

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

// return home function here.
void FlightPlanner::stepHome(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti)
{

    static int info_counter = 0;
  geometry_msgs::Vector3   localOffset;


  // set return home speed to the speed at which the mission was run.
  float homeSpeed = speedFactor ;
  float yawThresholdInDeg  = 2;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, home_start_gps_location );

  double xOffsetRemaining = home_target_offset_x - localOffset.x;
  double yOffsetRemaining = home_target_offset_y - localOffset.y;
  double zOffsetRemaining = home_target_offset_z - localOffset.z;
 

  double yawDesiredRad = Deg_To_Rad(target_yaw);
  double yawThresholdInRad = Deg_To_Rad(yawThresholdInDeg);
  double yawInRad          = toEulerAngle(current_atti).z;


info_counter++;
  if(info_counter > 50)
  {
    info_counter = 0;
    ROS_INFO( "HOME OFFSET: %f , %f, %f", localOffset.x, localOffset.y, localOffset.z);
     ROS_INFO("OFFSET LEFT  x: %f y: %f z: %f ", xOffsetRemaining, yOffsetRemaining, zOffsetRemaining);
  }

    if (abs(xOffsetRemaining) >= homeSpeed)
        xCmd = (xOffsetRemaining>0) ? homeSpeed : -1 * homeSpeed;
    else
        xCmd = xOffsetRemaining;

    if (abs(yOffsetRemaining) >= homeSpeed)
        yCmd = (yOffsetRemaining>0) ? homeSpeed : -1 * homeSpeed;
    else
        yCmd = yOffsetRemaining;

    if (abs(zOffsetRemaining) >= homeSpeed)
    zCmd = (zOffsetRemaining>0) ? homeSpeed : -1 * homeSpeed;
    else
    zCmd = zOffsetRemaining;

   

    /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */
    if (home_break_counter > 50)
    {
        ROS_INFO("BREAK_COUNTER GREATER THAN 50: ## %d", home_break_counter);
        homeReached = true;
        flightControl.land();
        return;
    }

    else if (home_break_counter > 0)
    {
        ROS_INFO_ONCE("Incrementing Break Counter for Home");
        droneControlSignal(0,0,0,0);
        home_break_counter++;
        return;
    }
    else //break_counter = 0, not in break stage
    {
       
        droneControlSignal(xCmd, yCmd, zCmd, yawDesiredRad, true, true);

    }

// Reduce speed
     if (std::abs(xOffsetRemaining) < 0.5 &&
        std::abs(yOffsetRemaining) < 0.5 && 
        std::abs(zOffsetRemaining) < 0.5 )
        {
            ROS_INFO_ONCE( "We are close to home.");
            homeSpeed = 1;
            
        }
   

    if (std::abs(xOffsetRemaining) < 0.5 &&
        std::abs(yOffsetRemaining) < 0.5 && 
        std::abs(zOffsetRemaining) < 0.5 )
        {
            ROS_INFO_ONCE( "We are very close to home.");
           
             home_inbound_counter ++;
        }

    else
    {
       if (home_inbound_counter != 0)
        {
            //! 2. Start incrementing an out-of-bounds counter
            home_outbound_counter ++;
        }
    }

    //! 3. Reset withinBoundsCounter if necessary
    if (home_outbound_counter > 10)
    {
        ROS_INFO( ": out of bounds, reset....");
        home_inbound_counter  = 0;
        home_outbound_counter = 0;
    }

    if (home_inbound_counter > 50)
    {
        ROS_INFO_ONCE("#####  start break....");
        home_break_counter = 1;
    }

}

// step between missions
void FlightPlanner::step(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti)
{
    static int info_counter = 0;
    geometry_msgs::Vector3     localOffset;

    float yawThresholdInDeg   = 2;

    float xCmd, yCmd, zCmd;

    localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

    double xOffsetRemaining = target_offset_x - localOffset.x;
    double yOffsetRemaining = target_offset_y - localOffset.y;
    double zOffsetRemaining = target_offset_z - localOffset.z;

    double yawDesiredRad = Deg_To_Rad(target_yaw);
    double yawThresholdInRad = Deg_To_Rad(yawThresholdInDeg);
    double yawInRad          = toEulerAngle(current_atti).z;
  
    if (abs(xOffsetRemaining) >= speedFactor)
        xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
    else
        xCmd = xOffsetRemaining;

    if (abs(yOffsetRemaining) >= speedFactor)
        yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
    else
        yCmd = yOffsetRemaining;

    if (abs(zOffsetRemaining) >= speedFactor)
    zCmd = (zOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
    else
    zCmd = zOffsetRemaining;

    /*!
    * @brief: if we already started breaking, keep break for 50 sample (1sec)
    *         and call it done, else we send normal command
    */
    if (break_counter > 50)
    {
        ROS_INFO("##### Route %d finished....", waypoint_index + 1);
        waypoint_finished = true;
        return;
    }

    else if (break_counter > 0)
    {
        ROS_INFO_ONCE("Incrementing Break Counter");
        droneControlSignal(0,0,0,0);
        break_counter++;
        return;
    }
    else 
    {
        droneControlSignal(xCmd, yCmd, zCmd, yawDesiredRad, true, true);
    }

    if (std::abs(xOffsetRemaining) < 0.5 &&
        std::abs(yOffsetRemaining) < 0.5 && 
        std::abs(zOffsetRemaining) < 0.5 )
        {
            //! 1. We are within bounds; start incrementing our in-bound counter
            ROS_INFO_ONCE( "We are close.");
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
        ROS_INFO( "##### Route %d: out of bounds, reset....", waypoint_index + 1);
        inbound_counter  = 0;
        outbound_counter = 0;
    }

    if (inbound_counter > 50)
    {
        ROS_INFO_ONCE("##### Route %d start break....", waypoint_index + 1);
        break_counter = 1;
    }

}


void FlightPlanner::prepareFlightPlan(double lat, double lon, double alt, unsigned char samplingTask)
{
    float altitude_offset = current_gps_location.altitude;
    sensor_msgs::NavSatFix flightWaypoint;
    flightWaypoint.latitude = lat;
    flightWaypoint.longitude = lon;
    flightWaypoint.altitude =  alt + altitude_offset; // add the offset from the ground to the drone's altitude
    unsigned char land = samplingTask;

    ROS_INFO("Atitude offset for waypoint %d set as %f, drone altitude is now at %f", waypoint_index, altitude_offset, flightWaypoint.altitude);

    appendFlightPlan(flightWaypoint, samplingTask);

    
}

 bool FlightPlanner::isMissionFinished()
 {
     // if waypoint count isn't zero and is equal to number of waypoint index, 
     // we can safely assume the mission is finished..

     if(waypoint_index >= waypoint_count)
     {
         ROS_INFO("Mission Waypoint count: %d", waypoint_count);
         ROS_INFO("Mission Finished Waypoint index %d", waypoint_index );
        return true;
     }

     else
     {
         ROS_INFO("Mission isn't finished yet. Next waypoint");
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

// This is very unstable and the implementation is probably incorrect so disregard this.
// Looking to use some sort of Model predictive control next.
// Need control knowledge gains
void FlightPlanner::droneControlSignalPID(double x, double y, double z, double yaw, bool use_yaw_rate , bool use_ground_frame )
{

    sensor_msgs::Joy controlPosYaw;

    x = pid.calculate(target_offset_x, current_local_position.x, -speedFactor, speedFactor );
    y = pid.calculate(target_offset_y, current_local_position.y, -speedFactor, speedFactor );
    z = pid.calculate(target_offset_z, current_local_position.z, -speedFactor, speedFactor );

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


void FlightPlanner::setWaypoint(sensor_msgs::NavSatFix newWaypoint)
{

    geometry_msgs::Vector3 offset_from_target;

   ROS_INFO("Waypoint COORDINATES #%d : %f  %f  %f", waypoint_index + 1, flight_plan[waypoint_index].latitude, flight_plan[waypoint_index].longitude, flight_plan[waypoint_index].altitude );

   localOffsetFromGpsOffset(offset_from_target, newWaypoint, start_gps_location );
   ROS_INFO("new Waypoint target offset  x: %f y: %f z: %f ", offset_from_target.x, offset_from_target.y, offset_from_target.z);

   // pass local offsets into global variable
   target_offset_x = offset_from_target.x;
   target_offset_y = offset_from_target.y;
   target_offset_z = offset_from_target.z;

}


bool FlightPlanner::reachedWaypoint()
{
    if(waypoint_finished)
    {
        ROS_INFO("We've reached waypoint %d", waypoint_index+1);
       return true;
    }

   else
   {
      return false;
   }

}

void FlightPlanner::appendFlightPlan(sensor_msgs::NavSatFix newWaypoint, unsigned char land)
{

    // add new waypoint to vector
    flight_plan.push_back(newWaypoint);

    // update number of waypoints
    waypoint_count = flight_plan.size();

    waypoint_lists.push(make_pair(flight_plan, land));

    ROS_INFO("Waypoint # %d added", waypoint_count);

    ROS_INFO ("Landing Task %d", land);

}

void FlightPlanner::onWaypointReached()
{ 
    // check if we are to land at the waypoint. 
    unsigned char checkTask = waypoint_lists.front().second;
    
    if(!waypoint_lists.empty())
    {   
        if(checkTask == 1)  
        {
            bool performTask;

            if(flightControl.check_M100())
            {
                performTask = flightControl.M100monitoredLanding();

                ROS_INFO_ONCE("Landing at the %dth waypoint", waypoint_index+1);

            }

            else
            {
                //performTask = flightControl.monitoredLanding();
                // This doesn't work on S1000 for now.
                // TODO: Would need to do some checks here to 
                flightControl.land();
            }

            if(performTask)
            {
                // Pause for a period of time here...
                // Integrate whatever task we're doing at this point.
                ros::Duration(10).sleep();
                flightControl.M100monitoredTakeoff();
            }

            else
            {
                ROS_ERROR("Unsucessful landing of drone");
            }
        }

    }

    waypoint_index++;

    // remove current waypoint from the list
    waypoint_lists.pop();

    state = MissionState::ARRIVED;
}

void FlightPlanner::onMissionFinished()
{

    ROS_INFO("We've finished the mission");
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
        deltaENU.z = target.altitude - origin.altitude ;

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
        switch(state)
        {
            case MissionState::IDLE:
            {
                if(waypoint_count != 0)
                {

                    inbound_counter = 0;
                    outbound_counter = 0;
                    break_counter = 0;
                    waypoint_finished = false;
                    setWaypoint(flight_plan[waypoint_index]);
                    
                    ROS_INFO("MISSION START ROUTE FROM IDLE %d / %d ", waypoint_index+1, waypoint_count);
                    
                    state = MissionState::NEW_WAYPOINT;
                    ROS_INFO("Next Index %d ", waypoint_index);

                }

                else
                {
                    ROS_INFO_THROTTLE(2, "Mission in idle state, waiting for a mission plan");
                }


                break;
            }

            case MissionState::NEW_WAYPOINT:
            {   
                // if  mission isn't finished, keep stepping through mission.
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

                    inbound_counter = 0;
                    outbound_counter = 0;
                    break_counter = 0;
                    waypoint_finished = false;
                    start_gps_location = current_gps_location;
                    start_local_position = current_local_position;
                    setWaypoint(flight_plan[waypoint_index]);
                   
                    ROS_INFO("MISSION START ROUTE ARRIVED %d / %d ", waypoint_index+1, waypoint_count);
                    state = MissionState::NEW_WAYPOINT;


                }

                else
                {
                    onMissionFinished();
                }

                break;
            }

            case MissionState::FINISHED:
            {
                ros::Duration(0.5).sleep();

                switch(checkMissionEnd)
                {

                     
                    case 3:
                    {

                        ROS_INFO("Computing home offsets");
                        geometry_msgs::Vector3 offset_from_home;
                        ROS_INFO(" HOME Waypoint COORDINATES :  %f ,  %f, %f ", home_gps_location.latitude, home_gps_location.longitude, home_gps_location.altitude );
                         localOffsetFromGpsOffset(offset_from_home, home_gps_location, current_gps_location );
                        ROS_INFO("HOME Waypoint target offset  x: %f y: %f z: %f ", offset_from_home.x, offset_from_home.y, offset_from_home.z);
                    
                        home_target_offset_x = offset_from_home.x;
                        home_target_offset_y = offset_from_home.y;
                        home_target_offset_z = offset_from_home.z;
                        // Set Starting point here.

                        home_start_gps_location = current_gps_location;                     
                        state = MissionState::GO_HOME;
                      break;
                    }

                    case 2:
                    {
                        flightControl.M100monitoredLanding();
                        state = MissionState::IDLE;   
                        break;
                    }

                    case 1:
                    {
                         droneControlSignal(0, 0, 0, 0, true, true);

                    }

                    default:
                    {
                         state = MissionState::IDLE;   
                         break;

                    }
                }

            }

            case MissionState::GO_HOME:
            {

               if(!homeReached)
               {

                    stepHome(current_gps_location, current_drone_attitude);
                   
               }
                break;

            }


            default:
            {
                break;
            }
        }
    
}



void FlightPlanner::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps_location = *msg;
  

  ROS_INFO_ONCE("GPS Location %f , %f , %f",  current_gps_location.latitude,  current_gps_location.longitude, current_gps_location.altitude);

}


void FlightPlanner::ekf_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_gps_location = *msg;

    ROS_INFO_ONCE("GPS Location %f , %f , %f",  current_gps_location.latitude,  current_gps_location.longitude, current_gps_location.altitude);
}

void FlightPlanner::local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    current_local_position = msg->point;

}

void FlightPlanner::ekf_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
   current_local_position = msg->pose.pose.position;

}

void FlightPlanner::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    current_drone_attitude = msg->quaternion;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Flight Control");

     FlightPlanner flight_planner;

     ROS_INFO("Running");

    ros::spin();

    return 0;
}

