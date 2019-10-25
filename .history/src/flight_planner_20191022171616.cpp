#include "m100_flight_planner/flight_planner.h"

Pid_control pid_pos, pid_yaw;
float kp;
float ki; 
float kd;

 int inbound_counter;
  int outbound_counter;
  int break_counter;

  ros::Publisher ctrlBrakePub;
  ros::Publisher ctrlPosYawPub;

char getch()
{
     int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

void FlightPlanner::keyboardControl()
{
    char c = getch();
    
    //ROS_INFO("The %d key was pressed", c);

    if(c!=EOF)
    {
       switch(c)
       {
           case '\e':
           {
               ROS_ERROR("Drone Emergency stop");
              droneControlSignal(0,0,0,0);  
              break; 
           }

           case 'c':
           {
               runMission();
               break;
           }
       }
    }

}

FlightPlanner::FlightPlanner()
{

    bool obtain_control;

    control_publisher = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

    // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
    
    mobile_data_subscriber = nh.subscribe<dji_sdk::MobileData>("dji_sdk/from_mobile_data", 10, &FlightPlanner::mobileDataSubscriberCallback, this);

    ROS_INFO("In control loop");

    obtain_control = obtainControl();
   
    if(obtain_control == true)
    {
        if(setLocalPosition())
        {
            ROS_INFO("Local Position set successfully! ");
        }
    }

            // initialise PID gains. 
    //TODO Use param server to change gains.
    yaw_limit = DegToRad(180);
    ROS_INFO("YAW LIMIT %f", yaw_limit );
    
      // set initial parameters. 

    state = MissionState::IDLE;
    waypoint_index = 0;
    waypoint_finished = false;
    yaw_flag = true;
    home_reached = false;
    
    target_position_vector = Eigen::Vector3d::Zero();
    target_yaw_angle = 0;
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

    if(!home_reached)
    {
        ROS_INFO_ONCE("On the way back home");
        stepHome();
    }  
 }

 void FlightPlanner::stepHome()
 {
    geometry_msgs::Vector3 offset_from_home;

    //ROS_INFO(" STEP Waypoint COORDINATES :  %f ,   %f", home_gps_location.latitude, home_gps_location.longitude );

    //getLocalPositionOffset(offset_from_home, home_gps_location, current_gps_location );
    getLocalPositionOffset(offset_from_home, current_gps_location, home_start_gps_location);

   // ROS_INFO("STEP Waypoint target offset  x: %f y: %f z: %f ", offset_from_home.x, offset_from_home.y, offset_from_home.z);

    // pass local offsets into global variable
    double home_x_offset_left = home_position_vector[0] - offset_from_home.x;
    double home_y_offset_left = home_position_vector[1] - offset_from_home.y;
    double home_z_offset_left =home_position_vector[2] -  offset_from_home.z;

    Eigen::Vector3d effort;
    effort << home_x_offset_left, home_y_offset_left, home_z_offset_left;
    ROS_INFO("STEP offset LEFT  x: %f y: %f z: %f ",home_x_offset_left,home_y_offset_left,home_z_offset_left);

    Eigen::Vector3d cmd_vector;
    cmd_vector = getHomeEffort(effort);

    double pid_effort = pid_pos.PIDupdate(home_distance);

    double x_cmd, y_cmd, z_cmd;

    x_cmd = pid_effort * cmd_vector[0];
    y_cmd = pid_effort * cmd_vector[1];
    z_cmd = pid_effort * cmd_vector[2];

    if(home_z_offset_left > 0.2)
    {
        droneControlSignal(0, 0, z_cmd, 0);
    }

    else
    {
        droneControlSignal(x_cmd, y_cmd, 0, 0, true, true);
    }
    

    if(hover_flag)
    {
       ROS_INFO("Drone Stop"); 
       droneControlSignal(0,0,0,0);

    }
   
   ROS_INFO("Distance to Home setpoint: %f",home_distance);
  // ROS_INFO("PID_Effort: %f", pid_effort);
   
   if(home_distance <= 1)
   {
        ROS_INFO("We are close");
        droneControlSignal(0,0,0,0);
        flightControl.land();
       home_reached = true;

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

             for(int i = 0; i < sizeof(sampling_time_array); i++)
             {
                 sampling_time_array [i] = data_from_mobile.data[i+22];
             }
             
   


            std::reverse(std::begin(latitude_array), std::end(latitude_array));
            std::reverse(std::begin(longitude_array), std::end(longitude_array));
            std::reverse(std::begin(altitude_array), std::end(altitude_array));
            std::reverse(std::begin(sampling_time_array), std::end(sampling_time_array));

            std::memcpy(&latitude, latitude_array, sizeof(double));
            std::memcpy(&longitude, longitude_array, sizeof(double));
            std::memcpy(&altitude, altitude_array, sizeof(float));
            std::memcpy(&samplingTime, sampling_time_array, sizeof(float));


            std::cout<< "Sampling at Waypoint for " << samplingTime << "seconds"  << std::endl;

            // Add waypoint to flight plan
            prepareFlightPlan(latitude, longitude, altitude, task, samplingTime);
             //prepareFlightPlan(latitude, longitude, altitude, task);
           break;

       }

       // Receive speed and mission END action
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

       // Clear Waypoints and set UAV to IDLE position
       // resets local position reference of the UAV
       case 0x3F:
       {
             
               flight_plan.clear();
               waypoint_count = 0;
               waypoint_index = 0;

               state = MissionState::IDLE;
               start_gps_location = current_gps_location;
               setLocalPosition();

               ROS_INFO("Waypoints cleared, Local Position Reference set, Aircraft now in IDLE State");

           break;        
       }

        // Land UAV
       case 0x03:
       {
           ROS_INFO("Received command to Land");
           flightControl.land();
           break;
       }

       // Takeoff UAV
       case 0x01:
       {
           ROS_INFO("Received command for takeoff");
           flightControl.M100monitoredTakeoff();
           break;
       }  

        // Abort Mission
        // UAV should hover in current position
       case 0x02:
       {

           ROS_INFO("Aborting mission: UAV Hovering");
           hover_flag = 1;

           break;
       }

       // Continue Mission
       case 0x5d:
       {
           ROS_INFO("continuing mission");
           hover_flag = 0;
           break;

       }

       // pause mission:
       case 0x5f:
       {
           ROS_INFO("Pausing Mission");
          hover_flag = 1;
          break;
       }

       // Start mission
       case 0x1A:
       {
            current_time = ros::Time::now();
            ros::Rate loop_rate(50);
           
            drone_version = checkM100(); // returns drone version is M100 if true

            // SET HOME GPS LOCATION here
            // This is the current position of the UAV before takeoff
            home_gps_location = current_gps_location;      
            ROS_INFO("Logged home GPS location at Lat:%f , Lon:%f", home_gps_location.latitude, home_gps_location.longitude);
          
            
            // Set Home GPS Altitude as a 5 metre offset 
            //from the current take off altitude of the drone
            home_gps_location.altitude = current_gps_location.altitude +  5.0;
            ROS_INFO("Logged home return GPS Altitude :%f ", home_gps_location.altitude);

            // check UAV Type to determine which Takeoff function to call
            if(drone_version)
                {
                    ROS_INFO("M100 Drone taking off");
                    takeoff_result = flightControl.M100monitoredTakeoff();        
                }
                else
                {

                    // For Drones using the A3/N3 variant
                    ROS_INFO("Custom Drone taking off");
                    takeoff_result = flightControl.monitoredTakeoff();
                }

                if(takeoff_result)
                {
                     
                    // Initialise PID 
                    ROS_INFO("Speed Factor %f", speedFactor);
                    pid_pos.PIDinit(0.5, 0, 0, speedFactor, -speedFactor);
                    pid_yaw.PIDinit(1, 0, 0, yaw_limit, -yaw_limit);

                    // set first gps position
                    if(drone_version)
                    {
                         setZOffset(current_local_position.z);
                    }
                   
                    start_gps_location = current_gps_location;
                    start_local_position = current_local_position;
                    
                    // Wait for Keyboard Press
                    ROS_INFO("Initiating mission Press C on your keyboard to start mission: ");
                   
                    std::cin.clear();
                    std::cin >> command;

                    std::cout << "command: " << command << std::endl;

                    // Start Mission
                    while(ros::ok() && command == "c")
                     {
                            ros::spinOnce();          
                            runMission();                         
                            loop_rate.sleep();
                        
                    }

                }

                break;
        }

        default:
        {
            break;
        }

    }

}

void FlightPlanner::setZOffset(double offset)
{
     z_offset_takeoff = offset;
     ROS_INFO("Z_offset applied as %f", z_offset_takeoff);
}

FlightPlanner::~FlightPlanner()
{
   
}

void FlightPlanner::step()
{
    geometry_msgs::Vector3 position_offset;

     // Compute the local position delta from the current waypoint to the next waypoint
      getLocalPositionOffset(position_offset, current_gps_location, start_gps_location);

    
    double x_offset_left = target_position_vector[0] - position_offset.x;
    double y_offset_left = target_position_vector[1] - position_offset.y;
    double z_offset_left = target_position_vector[2] - position_offset.z;

    //ROS_INFO("Z_offset_left: %f", z_offset_left );
    //ROS_INFO("y offset_left: %f", y_offset_left );
    //ROS_INFO("x_offset_left: %f", x_offset_left );
     Eigen::Vector3d effort;

     effort << x_offset_left, y_offset_left, z_offset_left;

     // To be used to set velocities on the UAV
     Eigen::Vector3d cmd_vector;

     cmd_vector = getEffort(effort);
    
     // Determine the UAV speed
     double pid_effort = pid_pos.PIDupdate(distance_to_setpoint);

     // commands to be sent to control publisher
     double x_cmd, y_cmd, z_cmd;
     x_cmd = pid_effort * cmd_vector[0];
     y_cmd = pid_effort * cmd_vector[1];
     z_cmd = pid_effort * cmd_vector[2];

        

    if(z_offset_left > 0.2)
    {
        ROS_INFO("Enter Z offset %f",z_cmd);
        // controlVelYawRate.axes.push_back(0);
        //  controlVelYawRate.axes.push_back(0);
        // controlVelYawRate.axes.push_back(z_cmd);
        //  controlVelYawRate.axes.push_back(0);
        // controlVelYawRate.axes.push_back(flag);
        //  ctrlBrakePub.publish(controlVelYawRate);
        droneControlSignal(0, 0, z_cmd, 0);
    }

    // send command to drone
    else
    {
       droneControlSignal(x_cmd, y_cmd, 0, 0);
    }
    
//    // ROS_INFO("PID_Effort: %f", pid_effort);
//    // ROS_INFO("Setpoint distance: %f", distance_to_setpoint);

    // If we are close, start braking
    if(distance_to_setpoint < 0.5)
    {
        droneControlSignal(0,0,0,0);
        waypoint_finished = true;
    }

    // // To play or pause mission 
    // if(hover_flag == true)
    // {
    //     ROS_INFO("Hover Flag");
    //    droneControlSignal(0,0,0,0);
    // }

}



void FlightPlanner::stepYaw()
{
     geometry_msgs::Vector3 position_offset;

    getLocalPositionOffset(position_offset, current_gps_location, start_gps_location);

    double x_offset_left = target_position_vector[0] - position_offset.x;
    double y_offset_left = target_position_vector[1] - position_offset.y;

       
    current_yaw_angle = toEulerAngle(current_drone_attitude).z;

    current_yaw_angle = (current_yaw_angle>=0) ? current_yaw_angle : current_yaw_angle + 2*C_PI;

    desired_yaw_angle = atan2(y_offset_left, x_offset_left);

    desired_yaw_angle = (desired_yaw_angle>=0) ? desired_yaw_angle : desired_yaw_angle + 2*C_PI;

    double yaw_diff = desired_yaw_angle - current_yaw_angle;

   // Yaw pid to be published to control signal.
    double yaw_pid = pid_yaw.PIDupdate(yaw_diff);

    double desired_yaw_angle_deg = RadToDeg(desired_yaw_angle) ;

    double current_yaw_deg =  RadToDeg(current_yaw_angle);

    ROS_INFO("Desired angle degree= %f", desired_yaw_angle_deg);

    ROS_INFO("Current Yaw Angle %f",current_yaw_deg );

    ROS_INFO("Yaw PID %f", yaw_pid);

    // Use Yaw angle
    droneControlSignal(0,0,0, yaw_pid, true, true);
    // Check if we are close to the required yaw.
    if(fabs(desired_yaw_angle_deg - current_yaw_deg) < 0.9 )    
    {
      yaw_flag = false;      
    }

}

float FlightPlanner::setYaw(float yaw)
{
    target_yaw_angle = yaw;

    return yaw;
}

Eigen::Vector3d FlightPlanner::setTarget(float x, float y, float z)
{
    ROS_INFO("Target set as %f, %f, %f", x , y , z);
    target_position_vector << x , y , z;

    return target_position_vector;

}

Eigen::Vector3d FlightPlanner::setHomeTarget(float x, float y, float z)
{
    ROS_INFO("Home point set as %f, %f, %f", x , y , z);  

    if(drone_version)  // if drone is M100
    {
      z = z - z_offset_takeoff;
      ROS_INFO("Removed Z Offset");
    }
    
    home_position_vector << x , y , z;

    return home_position_vector; 
}


Eigen::Vector3d FlightPlanner::getEffort(Eigen::Vector3d& target)
{
    double vector_length = target.norm();
    distance_to_setpoint = vector_length;

    // Initialise unit vector

    Eigen::Vector3d position_unit_vector = Eigen::Vector3d::Zero();

    // check for zero magnitude
    if(vector_length != 0)
    {
        // normalize vector;
        position_unit_vector = target.normalized();
    }

    return position_unit_vector;


}

Eigen::Vector3d FlightPlanner::getHomeEffort(Eigen::Vector3d& target)
{
    double vector_length = target.norm();
    home_distance = vector_length;

    // Initialise unit vector

    Eigen::Vector3d position_unit_vector = Eigen::Vector3d::Zero();

    // check for zero magnitude
    if(vector_length != 0)
    {
        // normalize vector;
        position_unit_vector = target.normalized();
    }

    return position_unit_vector;


}
void FlightPlanner::prepareFlightPlan(double lat, double lon, double alt, unsigned char sampling_task, float samplingTime)
{
    float altitude_offset = current_gps_location.altitude;
    sensor_msgs::NavSatFix flightWaypoint;
    flightWaypoint.latitude = lat;
    flightWaypoint.longitude = lon;
    flightWaypoint.altitude =  alt + altitude_offset; // add the UAV mission altitude offset from the ground to the drone's altitude
    
    ROS_INFO("Atitude offset for waypoint %d set as %f, drone altitude is now at %f", waypoint_index, altitude_offset, flightWaypoint.altitude);
     ROS_INFO("Latitude: %f, Longitude: %f", flightWaypoint.latitude, flightWaypoint.longitude);


    // Add UAV Waypoint to the 
    appendFlightPlan(flightWaypoint, sampling_task, samplingTime);

    
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

   if (use_yaw_rate) // using yaw rate and body frame
    {
         controlPosYaw.axes.push_back(control_flag_fru); 
    }

    if (!use_yaw_rate && use_ground_frame) // using yaw angle and ground frame
    {
         controlPosYaw.axes.push_back(control_flag_yaw_angle_enu); 
    }

    else // using yaw angle and UAV Body frame
    {
         controlPosYaw.axes.push_back(control_flag_yaw_angle_fru); 
    }

    control_publisher.publish(controlPosYaw);
}


void FlightPlanner::setWaypoint(sensor_msgs::NavSatFix newWaypoint)
{

   geometry_msgs::Vector3 offset_from_target;

   ROS_INFO("Waypoint COORDINATES #%d : %f  %f  %f", waypoint_index + 1, flight_plan[waypoint_index].latitude, flight_plan[waypoint_index].longitude, flight_plan[waypoint_index].altitude );

   getLocalPositionOffset(offset_from_target, newWaypoint, start_gps_location );
   ROS_INFO("new Waypoint target offset  x: %f y: %f z: %f ", offset_from_target.x, offset_from_target.y, offset_from_target.z);

   // set Target 
   setTarget(offset_from_target.x, offset_from_target.y, offset_from_target.z);

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

void FlightPlanner::appendFlightPlan(sensor_msgs::NavSatFix newWaypoint, unsigned char land, float samplingTime)
{

    // add new waypoint to vector
    flight_plan.push_back(newWaypoint);

    // update number of waypoints
    waypoint_count = flight_plan.size();
        
    waypoint_lists.push(std::make_tuple(flight_plan, land, samplingTime));
   // waypoint_lists.push(make_pair(flight_plan, land));

    ROS_INFO("Waypoint # %d added", waypoint_count);
    auto data = waypoint_lists.front();
    std::vector<sensor_msgs::NavSatFix> GPS = std::get<0>(data);
    unsigned char checkTask = std::get<1>(data);
    float sampleTime = std::get<2>(data);




    ROS_INFO ("Landing Task %d", checkTask);
     ROS_INFO ("Landing Task %f", sampleTime);
     ROS_INFO("Waypoint Lat: %f, Lon%f: ", GPS.front().latitude, GPS.front().longitude);

}

/*
Callbacks for the flight anomalies reported by the drone
Implement detailed safety routines here
Only Support A3/N3 
*/
void FlightPlanner::flightAnomalyCallback(const dji_sdk::FlightAnomaly::ConstPtr & msg)
{
    uint32_t flightAnomalydata = msg->data;

    if(flightAnomalydata)
    {
        // WHat should we do fo each use case?
        ROS_ERROR("Flight Anomaly detected by the UAV:" );
        if(flightAnomalydata && dji_sdk::FlightAnomaly::IMPACT_IN_AIR)
        {
            ROS_ERROR("UAV Struck an objkect in air");
            // Define what should be done here
        }

        if(flightAnomalydata && dji_sdk::FlightAnomaly::RANDOM_FLY)
        {
            // Pause mission and let drone just stop in air. 
            ROS_ERROR("UAV randomly implemented flight routine without command. Pausing Mission");
            droneControlSignal(0, 0, 0, 0, true, true);
        }

        if(flightAnomalydata && dji_sdk::FlightAnomaly::VERTICAL_CONTROL_FAIL)
        {
            ROS_ERROR("UAV has lost Height Control, Drone landing ");
            flightControl.land();
            
        }

        if(flightAnomalydata && dji_sdk::FlightAnomaly::HORIZONTAL_CONTROL_FAIL)
        {
            ROS_ERROR("UAV has lost roll/pitch control, Hovering in the air");
            droneControlSignal(0,0,0,0, true, true);
        }

        if(flightAnomalydata && dji_sdk::FlightAnomaly::YAW_CONTROL_FAIL)
        {
            ROS_ERROR("UAV has lost yaw control" );
            // do something here....
            // We don't essentially want to  stop anything if we can't yaw but 
            // revisit this aspect later on I guess.
        }

        if(flightAnomalydata && dji_sdk::FlightAnomaly::AIRCRAFT_IS_FALLING)
        {
            ROS_ERROR("Aircraft is falling");
            // Well, we're fucked innit?
            // possibly try to stabilise?
            droneControlSignal(0,0,0,0);
        }

        if(flightAnomalydata && dji_sdk::FlightAnomaly::STRONG_WIND_LEVEL1)
        {
            ROS_INFO("Strong Winds of LEVEL 1, Fly with caution");

        }

        if(flightAnomalydata && dji_sdk::FlightAnomaly::STRONG_WIND_LEVEL2)
        {
            ROS_INFO("Strong Winds of LEVEL 2, Drone flying back home");
            stepHome(); 
            
        }
         if(flightAnomalydata && dji_sdk::FlightAnomaly::COMPASS_INSTALLATION_ERROR)
        {
            ROS_ERROR("Compass Installed incorrectly, Aircraft will land");
            // Well, we're fucked innit?
            // possibly try to stabilise?
            flightControl.land();
        }

        if(flightAnomalydata && dji_sdk::FlightAnomaly::IMU_INSTALLATION_ERROR)
        {
            ROS_ERROR("IMU INSTALLATION ERROR, LANDING");
            flightControl.land();

        }

        if(flightAnomalydata && dji_sdk::FlightAnomaly::ESC_TEMPERATURE_HIGH)
        {
            ROS_INFO("ESC Temperature too high, keep an eye on this");

    
            
            
        }
        if(flightAnomalydata && dji_sdk::FlightAnomaly::ESC_DISCONNECTED)
        {
            ROS_ERROR("ESC DIsconnected, land drone now");
            flightControl.land();
            
            
        }
        if(flightAnomalydata && dji_sdk::FlightAnomaly::GPS_YAW_ERROR)
        {
            ROS_ERROR("GPS Yaw ERROR");
            // Not sure how to handle this...
            
            
            
        }


    }

}

void FlightPlanner::onWaypointReached()
{ 
    // check if we are to land at the waypoint. 
     //unsigned char checkTask = waypoint_lists.front().second;
    auto waypoint_settings = waypoint_lists.front();
    unsigned char checkTask = std::get<1>(waypoint_settings);
    float sampleTime = std::get<2>(waypoint_settings);

    
    if(!waypoint_lists.empty())
    {   
        if(checkTask == 1)  
        {
            bool performTask;

            if(checkM100())
            {
                performTask = flightControl.M100monitoredLanding();

                ROS_INFO_ONCE("Landing at the %dth waypoint", waypoint_index+1);

            }

            else
            {
                performTask = flightControl.monitoredLanding();
                // This doesn't work on S1000 for now.
                // TODO: Would need to do some checks here to 
                //flightControl.land();
            }

            if(performTask)
            {
                // Pause for a period of time here...
                // Integrate whatever task we're doing at this point.
                // ToDO iintegrate UART_START_DMA Here.

                if(drone_version)
                {
                     ros::Duration(2).sleep();
                    
                    if(waypoint_lists.size()> 1) // Only take off if we're not at the last waypoint and mission end is to autoland
                    {
                         flightControl.M100monitoredTakeoff();
                    }

                    if(waypoint_lists.size()== 1 && checkMissionEnd == 3) // if we need to fly back home
                    {
                         flightControl.M100monitoredTakeoff();
                    }
                   
                }

                else
                {
                   ROS_INFO("in landing loop");
                    
                        ROS_INFO("Drone Entering Sleeep");
                        ros::Duration(10).sleep();
                        ROS_INFO("Drone Exited Sleeep");
                        flightControl.monitoredTakeoff();
                   
                }           
               
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
    
    // Update State to UAV has arrived at the waypoint
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
    // UART_STOP_DMA

}
void FlightPlanner::getLocalPositionOffset(geometry_msgs::Vector3& deltaENU, sensor_msgs::NavSatFix& target, sensor_msgs::NavSatFix& origin)
{
        double deltaLon = target.longitude - origin.longitude;
        double deltaLat = target.latitude - origin.latitude;

        deltaENU.y = DegToRad(deltaLat) * C_EARTH;
        deltaENU.x = DegToRad(deltaLon) * C_EARTH * cos(DegToRad(target.latitude));
        deltaENU.z = target.altitude - origin.altitude ;

}


void FlightPlanner::runMission()
{
        switch(state)
        {
            case MissionState::IDLE:
            {
                if(waypoint_count != 0) //TODO Fix waypoint mission from idle bug
                {

                    waypoint_finished = false;
                    yaw_flag = true;
                    setWaypoint(flight_plan[waypoint_index]);
                    
                    ROS_INFO("MISSION START ROUTE FROM IDLE %d / %d ", waypoint_index+1, waypoint_count);
                    
                    state = MissionState::NEW_WAYPOINT;
                    ROS_INFO("Next Index %d ", waypoint_index);

                }

                else
                {
                    ROS_INFO_THROTTLE(2, "Mission in idle state, waiting for a mission plan");
                    // Set position reference here
                    start_gps_location = current_gps_location;
                    setLocalPosition(); // Should only be used when a drone lands on the ground!!!
                }


                break;
            }

            case MissionState::NEW_WAYPOINT:
            {   
                // if  mission isn't finished, keep stepping through mission.
                if(!reachedWaypoint())
                {
                    // YAW 
                      if(yaw_flag)
                      {
                          stepYaw();
                      }

                      if(!yaw_flag)
                      {
                         step();
                      }

                   
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
                    waypoint_finished = false;
                    yaw_flag = true;
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
                ros::Duration(0.1).sleep();

                ROS_INFO ("Drone is case %d", checkMissionEnd);

                switch(checkMissionEnd)
                {
                    case 1:
                    {
                         ROS_INFO("Hovering at position ");
                         droneControlSignal(0, 0, 0, 0, true, true);
                         break;

                    }

                    case 2:
                    {
                        // if(drone_version)
                        // {
                        //     flightControl.M100monitoredLanding();
                        // }
                        // else
                        // {
                        //     flightControl.land();
                        // }
                        
                        
                        state = MissionState::IDLE;   
                        break;
                    }

                     
                    case 3:
                    {

                       // ROS_INFO("Computing home offsets");
                        geometry_msgs::Vector3 offset_from_home;
                       // ROS_INFO(" HOME Waypoint COORDINATES :  %f ,  %f, %f ", home_gps_location.latitude, home_gps_location.longitude, home_gps_location.altitude );
                       getLocalPositionOffset(offset_from_home, home_gps_location, current_gps_location );
                        //ROS_INFO("HOME Waypoint target offset  x: %f y: %f z: %f ", offset_from_home.x, offset_from_home.y, offset_from_home.z);
                    
                        setHomeTarget(offset_from_home.x, offset_from_home.y, offset_from_home.z);

                        home_start_gps_location = current_gps_location;                     
                        
                        if(!home_reached)
                         {
                            stepHome();
                            
                         }

                         ROS_INFO("Home reached %d", home_reached);

                      break;
                    }              

                    default:
                    {
                         state = MissionState::IDLE;   
                         break;

                    }
                }

            }

            default:
            {
                break;
            }
        }
    
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "Flight Control");

     FlightPlanner flight_planner;

     ROS_INFO("Running");

    ros::spin();

    return 0;
}