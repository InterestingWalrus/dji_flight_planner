#include "m100_flight_planner/flight_planner.h"

FlightPlanner::FlightPlanner()
{
    control_pub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
    gps_sub = nh.subscribe("/dji_sdk/gps_position", 10, &FlightPlanner::gps_callback, this);
    local_position_sub = nh.subscribe("/dji_sdk/local_position", 10, &FlightPlanner::local_position_callback, this);
    attitude_sub = nh.subscribe("/dji_sdk/attitude", 10, &FlightPlanner::attitude_callback, this);
    mobile_data_subscriber = nh.subscribe<dji_sdk::MobileData>("dji_sdk/from_mobile_data", 10, &FlightPlanner::mobileDataSubscriberCallback, this);

    FlightControl flightControl;
    PID pid;

    ROS_INFO("In loop");

    obtain_control = flightControl.obtainControl();
   
    if(obtain_control == true)
    {
        if(flightControl.set_local_position())
        {
            ROS_INFO("Local Position set successfully! ");
        }
    }

    state = MissionState::IDLE;
    waypoint_finished = false;
    waypoint_index = 0;
    waypoint_count = 0;
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;
    target_yaw = 50;

     state_1 = 0;

     speedFactor = 5;

    sensor_msgs::NavSatFix waypoint1;
    sensor_msgs::NavSatFix waypoint2;
    sensor_msgs::NavSatFix waypoint3;
    sensor_msgs::NavSatFix waypoint4;
    sensor_msgs::NavSatFix waypoint5;
    sensor_msgs::NavSatFix waypoint6;
    sensor_msgs::NavSatFix waypoint7;
    sensor_msgs::NavSatFix waypoint8;
    sensor_msgs::NavSatFix waypoint9;
    sensor_msgs::NavSatFix waypoint10;
    sensor_msgs::NavSatFix waypoint11;

    // waypoint1.latitude = 52.762018;
    // waypoint1.longitude  = -1.240814;
    // waypoint1.altitude = 10;

    // waypoint1.latitude = 52.762905;
    // waypoint1.longitude  = -1.240814;
    // waypoint1.altitude = 10;


    waypoint1.latitude = 52.756187;
    waypoint1.longitude  = -1.246298;
    waypoint1.altitude = 100;


    waypoint2.latitude = 52.762199 ;   
    waypoint2.longitude  = -1.240455;
    waypoint2.altitude = 10;

    waypoint3.latitude =  52.762391;
    waypoint3.longitude  = -1.240010;
    waypoint3.altitude = 10;
   

    waypoint4.latitude = 52.762571; 
    waypoint4.longitude  = -1.239638;
    waypoint4.altitude = 10;

    waypoint5.latitude =  52.762905;
    waypoint5.longitude = -1.238920;
    waypoint5.altitude = 10;

    waypoint6.latitude = 52.763293;  
    waypoint6.longitude  = -1.238135;
    waypoint6.altitude = 10;

    waypoint7.latitude =  52.763514; 
    waypoint7.longitude  = -1.237585;
    waypoint7.altitude = 10;

    waypoint8.latitude =  52.762837; 
    waypoint8.longitude  = -1.238250;
    waypoint8.altitude = 10;

    waypoint9.latitude =  52.762660; 
    waypoint9.longitude  = -1.238014;
    waypoint9.altitude = 10;

    waypoint10.latitude = 52.762449 ;
    waypoint10.longitude  = -1.237724;
    waypoint10.altitude = 10;


  appendFlightPlan(waypoint1);
//   appendFlightPlan(waypoint2);
//   appendFlightPlan(waypoint3);
//   appendFlightPlan(waypoint4);
//   appendFlightPlan(waypoint5);
//   appendFlightPlan(waypoint6);
//    appendFlightPlan(waypoint7);
//   appendFlightPlan(waypoint8);
//   appendFlightPlan(waypoint9);
//   appendFlightPlan(waypoint10);

   // run mission.

    ros::Rate loop_rate(50);
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
                start_gps_location = current_gps_location;
                 start_local_position = current_local_position;
                
                //state = MissionState::NEW_WAYPOINT;

              //  setWaypoint(flight_plan[waypoint_index]);

                //waypoint_index ++;

               

                ROS_INFO("Initiating mission");

                while(ros::ok())
                {
                    ros::spinOnce();
                    runMission();

                    loop_rate.sleep();
                }


            }
        
    
}

void FlightPlanner::mobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& mobile_data)
{

    data_from_mobile = *mobile_data;
    unsigned char data;
    memcpy(&data, &data_from_mobile.data[0], 10);

    unsigned char flight_data[20] = {0};

    ROS_INFO_STREAM ("Receieved from mobile: " <<std::hex << data_from_mobile );

    unsigned char CMD = data_from_mobile.data[0];

    double latitude;
    double longitude;
    float altitude;

    switch(CMD)
    {
       case 0x01:
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
   


            std::reverse(std::begin(latitude_array), std::end(latitude_array));
            std::reverse(std::begin(longitude_array), std::end(longitude_array));
            std::reverse(std::begin(altitude_array), std::end(altitude_array));

            std::memcpy(&latitude, latitude_array, sizeof (double));
            std::memcpy(&longitude, longitude_array, sizeof (double));
            std::memcpy(&altitude, altitude_array, sizeof(float));


            prepareFlightPlan(latitude, longitude, altitude);


           break;


       }

       case 0x1A:
       {
           // Clear all points. Use onMissionFinished for now
           onMissionFinished();

           ROS_INFO ("Waypoints cleared");
           break;
       }

       case 0x2A:
       {
           // run mission.
        //    if(flightControl.check_M100())
        //     {
        //         ROS_INFO("M100 Drone taking off");
        //         takeoff_result = flightControl.M100monitoredTakeoff();

        //     }
        //     else
        //     {

        //         // Drone is an A3/N3 variant
        //         ROS_INFO("Custom Drone taking off");
        //         takeoff_result = flightControl.monitoredTakeoff();

        //     }

        //     if(takeoff_result)
        //     {
        //         reset(current_gps_location, current_local_position);

        //         ROS_INFO("Initiating mission");

        //         while(ros::ok())
        //         {
        //             ros::spinOnce();
        //             runMission();
        //         }

        //     }

        //     else
        //     {
        //         ROS_INFO("Drone failed to take off");
        //     }

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

void FlightPlanner::step(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti)
{
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

//   float speedFactor         = 15;
  float yawThresholdInDeg   = 2;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

 // double yawDesiredRad     = Deg_To_Rad(target_yaw);  // This is shit...
  double yawDesiredRad = Deg_To_Rad(target_yaw);
  double yawThresholdInRad = Deg_To_Rad(yawThresholdInDeg);
  double yawInRad          = toEulerAngle(current_atti).z;
  
  // double yawDesiredRad      = atan2(yOffsetRemaining, xOffsetRemaining);

info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    //ROS_INFO( "-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
    //ROS_INFO( "+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);

    //ROS_INFO( "YAW in RAD: %f",yawInRad);

   // ROS_INFO( "YAW DESIRED RAD: % f", yawDesiredRad);
    //ROS_INFO( "inbound_counter %d", inbound_counter );
  }

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

    //zCmd = start_local_position.z + target_offset_z;

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
       // droneControlSignalPID(0, 0, 0, 0);
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
        //droneControlSignalPID(xCmd, yCmd, zCmd, yawDesiredRad, true, true);

    }

    if (std::abs(xOffsetRemaining) < 0.5 &&
        std::abs(yOffsetRemaining) < 0.5 && 
        std::abs(zOffsetRemaining) < 0.5 &&
        std::abs((yawInRad - yawDesiredRad) < yawThresholdInRad))
        {
            //! 1. We are within bounds; start incrementing our in-bound counter
            //ROS_INFO_THROTTLE(1, "We are close.");
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


void FlightPlanner::prepareFlightPlan(double lat, double lon, double alt)
{
    sensor_msgs::NavSatFix flightWaypoint;
    flightWaypoint.latitude = lat;
    flightWaypoint.longitude = lon;
    flightWaypoint.altitude =  alt;


    appendFlightPlan(flightWaypoint);

    
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


void FlightPlanner::droneControlSignalPID(double x, double y, double z, double yaw, bool use_yaw_rate , bool use_ground_frame )
{

    sensor_msgs::Joy controlPosYaw;

    x = pid.calculate(target_offset_x, current_local_position.x, -speedFactor, speedFactor );
    y = pid.calculate(target_offset_y, current_local_position.y, -speedFactor, speedFactor );
    z = pid.calculate(target_offset_z, current_local_position.z, -speedFactor, speedFactor );
   // sampleX = x;
   // sampleY = y;

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

// void FlightPlanner::reset(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Point &current_local_pos)
// {
    
//     inbound_counter = 0;
//     outbound_counter = 0;
//     break_counter = 0;

//     bool waypoint_finished = false;
    
//     state = MissionState::NEW_WAYPOINT;
//     ROS_INFO("waypoint index: %d", waypoint_index);

//       // increment waypoint to next waypoint
      
//     if (waypoint_index+1 < flight_plan.size()) 
//     {
//         waypoint_index++;
//         setWaypoint(flight_plan[waypoint_index]);
//         ROS_INFO("Set new waypoint %d / %d ", waypoint_index , waypoint_count );
//         start_gps_location = current_gps;
//         start_local_position = current_local_pos;
//     }

//     else
//     {
//         ROS_INFO("Out of bounds");
//       //  ROS_INFO("waypoint out of bounds # %d", waypoint_index );
//        // ROS_INFO("Flight plan count %d", flight_plan.size() );
//     }

// }


void FlightPlanner::setWaypoint(sensor_msgs::NavSatFix newWaypoint)
{
   geometry_msgs::Vector3 offset_from_target;

   // get GPS offset

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

             ROS_INFO_ONCE("Landing at the %dth waypoint", waypoint_index+1);

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

    waypoint_index++;

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

        // ROS_INFO("delta LAt:  %f", deltaLat);

        // ROS_INFO("delta LON:  %f", deltaLon);

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
        switch(state)
        {
            case MissionState::IDLE:
            {
                if(waypoint_count != 0)
                {
                    // reset mission and set waypoint
                    // NEW WAYPOINT flag is set in reset function
                   // ROS_INFO("Waypoint index MISSION IDLE: %d", waypoint_index);
                   // reset(current_gps_location, current_local_position);
                    inbound_counter = 0;
                    outbound_counter = 0;
                    break_counter = 0;
                    waypoint_finished = false;
                    //start_gps_location = current_gps_location;
                    //start_local_position = current_local_position;
                    setWaypoint(flight_plan[waypoint_index]);
                    
                    ROS_INFO("MISSION START ROUTE FROM IDLE %d / %d ", waypoint_index+1, waypoint_count);
                    
                    state = MissionState::NEW_WAYPOINT;
                   // waypoint_index++;
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
                //ROS_INFO("New waypoint");
                // if if mission isn't finished, keep stepping through mission.
               // ROS_INFO("Waypoint index MISSION NEW: %d", waypoint_index);
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
                    //reset(current_gps_location, current_local_position);
                    inbound_counter = 0;
                    outbound_counter = 0;
                    break_counter = 0;
                    waypoint_finished = false;
                    start_gps_location = current_gps_location;
                    start_local_position = current_local_position;
                    setWaypoint(flight_plan[waypoint_index]);
                   
                    ROS_INFO("MISSION START ROUTE ARRIVED %d / %d ", waypoint_index+1, waypoint_count);
                    state = MissionState::NEW_WAYPOINT;
                    // waypoint_index++;

                }

                else
                {
                    onMissionFinished();
                }

                break;
            }

            case MissionState::FINISHED:
            {

               ROS_INFO_ONCE("End of Mission");

               //flightControl.land(); // use this for now....

              ros::Duration(0.5).sleep();

              flightControl.M100monitoredLanding();
 
              // state = MissionState::IDLE;
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


void FlightPlanner::local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    current_local_position = msg->point;

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

