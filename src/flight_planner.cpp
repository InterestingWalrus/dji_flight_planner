#include "m100_flight_planner/flight_planner.h"

int inbound_counter;
int outbound_counter;
int break_counter;

char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
    {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
    {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0)
    {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
    {
        perror("tcsetattr ~ICANON");
    }
    return (buf);
}

void FlightPlanner::keyboardControl()
{
    char c = getch();

    if (c != EOF)
    {
        switch (c)
        {
        case '\e': // Escape
        {
            ROS_ERROR("Drone Emergency stop");
            droneControlSignal(0, 0, 0, 0);
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

    error_publisher = nh.advertise<std_msgs::Float32>("dji_sdk/landing_error", 10);

    mobile_data_subscriber = nh.subscribe<dji_sdk::MobileData>("dji_sdk/from_mobile_data", 10, &FlightPlanner::mobileDataSubscriberCallback, this);

    ROS_INFO("In control loop");

    obtain_control = obtainControl();

    if (obtain_control == true)
    {
        if (setLocalPosition())
        {
            ROS_INFO("Local Position set successfully! ");
        }
    }

    //TODO Use param server to change gains.
    yaw_limit = DegToRad(180);
    ROS_INFO("YAW LIMIT %f", yaw_limit);

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
    hover_flag = 0;
    verti_control = 1;

    ki = 0;
    kp = 0.5;
    kd = 0;

    kp_z = 0.8;
    ki_z = 0;
    kd_z = 0.001;

    kp_y = 1.2;
    ki_y = 0;
    kd_y = 0.001;
}

void FlightPlanner::stepHome()
{
    geometry_msgs::Vector3 offset_from_home;

    //ROS_INFO_ONCE(" STEP Waypoint COORDINATES :  %f ,   %f", home_gps_location.latitude, home_gps_location.longitude );

    getLocalPositionOffset(offset_from_home, current_gps_location, home_start_gps_location);

    // ROS_INFO("STEP Waypoint target offset  x: %f y: %f z: %f ", offset_from_home.x, offset_from_home.y, offset_from_home.z);

    // pass local offsets into global variable
    double home_x_offset_left = home_position_vector[0] - offset_from_home.x;
    double home_y_offset_left = home_position_vector[1] - offset_from_home.y;
    double home_z_offset_left = home_position_vector[2] - offset_from_home.z;

    Eigen::Vector3d effort;
    effort << home_x_offset_left, home_y_offset_left, home_z_offset_left;
    // ROS_INFO("STEP offset LEFT  x: %f y: %f z: %f ", home_x_offset_left, home_y_offset_left, home_z_offset_left);

    Eigen::Vector3d cmd_vector;
    cmd_vector = getHomeEffort(effort);
    double temp_val = home_target_norm - home_distance;

    double pid_effort = pid_pos.update(home_target_norm, temp_val, dt);

    double x_cmd, y_cmd, z_cmd;

    x_cmd = pid_effort * cmd_vector[0];
    y_cmd = pid_effort * cmd_vector[1];
    z_cmd = pid_effort * cmd_vector[2];

    if (home_z_offset_left > 1)
    {
        droneControlSignal(0, 0, z_cmd, 0);
    }

    else
    {
        droneControlSignal(x_cmd, y_cmd, 0, 0, true, true);
    }

    if (hover_flag)
    {
        ROS_INFO("Drone Stop");
        droneControlSignal(0, 0, 0, 0);
    }

    //ROS_INFO("Distance to Home setpoint: %f", home_distance);

    //TODO: Fix this and fix overshhot on RTH. Update to 3.9 and use FC class to set RTH
    if (home_distance <= 2)
    {
        ROS_INFO("We are close");
        droneControlSignal(0, 0, 0, 0);
        flightControl.land();
        home_reached = true;
    }
}

void FlightPlanner::mobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr &mobile_data)
{
    data_from_mobile = *mobile_data;
    unsigned char data;
    memcpy(&data, &data_from_mobile.data[0], 10);
    unsigned char flight_data[28] = {0};
    unsigned char CMD = data_from_mobile.data[0];

    switch (CMD)
    {
    case 0x2f:
    {
        ROS_INFO("Waypoints received");

        for (int i = 0; i < sizeof(latitude_array); i++)
        {
            latitude_array[i] = data_from_mobile.data[i + 1];
        }

        for (int i = 0; i < sizeof(longitude_array); i++)
        {
            longitude_array[i] = data_from_mobile.data[i + 9];
        }

        for (int i = 0; i < sizeof(altitude_array); i++)
        {
            altitude_array[i] = data_from_mobile.data[i + 17];
        }

        task = data_from_mobile.data[21];

        for (int i = 0; i < sizeof(sampling_time_array); i++)
        {
            sampling_time_array[i] = data_from_mobile.data[i + 22];
        }

        std::reverse(std::begin(latitude_array), std::end(latitude_array));
        std::reverse(std::begin(longitude_array), std::end(longitude_array));
        std::reverse(std::begin(altitude_array), std::end(altitude_array));
        std::reverse(std::begin(sampling_time_array), std::end(sampling_time_array));

        std::memcpy(&latitude, latitude_array, sizeof(double));
        std::memcpy(&longitude, longitude_array, sizeof(double));
        std::memcpy(&altitude, altitude_array, sizeof(float));
        std::memcpy(&samplingTime, sampling_time_array, sizeof(float));

        std::cout << "Sampling at Waypoint for " << samplingTime << "seconds" << std::endl;

        // Add waypoint to flight plan
        prepareFlightPlan(latitude, longitude, altitude, task, samplingTime);

        break;
    }

    // Receive speed and mission END action
    case 0x4d:
    {
        ROS_INFO("Flight Parameters received");

        for (int i = 0; i < sizeof(speed_array); i++)
        {
            speed_array[i] = data_from_mobile.data[i + 1];
        }

        missionEnd = data_from_mobile.data[5];

        std::reverse(std::begin(speed_array), std::end(speed_array));

        std::memcpy(&speedFactor, speed_array, sizeof(float));

        switch (missionEnd)
        {
        case 1: // NO ACTION
            checkMissionEnd = 1;
            std::cout << "HOVERING ACTION" << std::endl;
            break;

        case 2: //Return Home
            checkMissionEnd = 3;
            std::cout << "RETURN HOME" << std::endl;

            break;

        case 3: // Monitored Landing
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
        home_gps_location.altitude = current_gps_location.altitude + 5.0;
        ROS_INFO("Logged home return GPS Altitude :%f ", home_gps_location.altitude);

        // check UAV Type to determine which Takeoff function to call
        if (drone_version)
        {
            ROS_INFO("M100 Drone taking off");
            takeoff_result = flightControl.M100monitoredTakeoff();
        }
        else
        {

            // For Drones using the A3/N3 Autopilot
            ROS_INFO("Custom Drone taking off");
            takeoff_result = flightControl.monitoredTakeoff();
        }

        if (takeoff_result)
        {
            // Initialise PID
            //TODO: Set different gains for both drones
            ROS_INFO("Speed Factor %f", speedFactor);

            if (speedFactor < z_max)
            {
                z_clamp_p = speedFactor;
            }

            else
            {
                z_clamp_p = z_max;
            }

            if (z_min > -speedFactor)
            {
                z_clamp_n = z_min;
            }

            else
            {
                z_clamp_n = -speedFactor;
            }
            pid_pos.init(kp, ki, kd, speedFactor, -speedFactor);
            pid_z.init(kp_z, ki_z, kd_z, 5, z_clamp_n);
            pid_yaw.init(kp_y, ki_y, kd_y, speedFactor, -speedFactor);
            // set first gps position
            // if (drone_version)
            // {
            //     setZOffset(current_local_position.z);
            // }

            start_gps_location = current_gps_location;
            start_local_position = current_local_position;

            //Wait for Keyboard Press
            ROS_INFO("Initiating mission Press C on your keyboard to start mission: ");

            // std::cin.clear();
            // std::cin >> command;
            // std::cout << "command: " << command << std::endl;

            //Start Mission
            // while (ros::ok() && command == "c")
            while (ros::ok())
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

Eigen::Vector2d FlightPlanner::getHorizontalEffort(Eigen::Vector2d &target)
{
    double vector_length = target.norm();
    xy_setpoint_dist = vector_length;

    Eigen::Vector2d position_unit_vector = Eigen::Vector2d::Zero();

    if (vector_length != 0)
    {
        position_unit_vector = target.normalized();
    }

    return position_unit_vector;
}

void FlightPlanner::step()
{
    geometry_msgs::Vector3 position_offset;

    // Compute the local position delta from the current waypoint to the next waypoint
    getLocalPositionOffset(position_offset, current_gps_location, start_gps_location);

    double x_offset_left = target_position_vector[0] - position_offset.x;
    double y_offset_left = target_position_vector[1] - position_offset.y;
    double z_offset_left = target_position_vector[2] - position_offset.z - z_offset_takeoff;
    ;

    Eigen::Vector2d xy_effort;
    xy_effort << x_offset_left, y_offset_left;
    Eigen::Vector2d cmd_vector;
    cmd_vector = getHorizontalEffort(xy_effort);

    double temp_val = hori_target_norm - xy_setpoint_dist;
    double pid_effort = pid_pos.update(hori_target_norm, temp_val, dt);

    double z_pid_effort = pid_z.update(target_position_vector[2], current_local_position.z, dt);

    double x_cmd, y_cmd, z_cmd;
    x_cmd = pid_effort * cmd_vector[0];
    y_cmd = pid_effort * cmd_vector[1];
    z_cmd = z_pid_effort;

    // Wait for UAV to gain required altitude
    if (verti_control == 1 && z_offset_left > 0.1)
    {
        //ROS_INFO("Z Offset: %f", z_offset_left);
        droneControlSignal(0, 0, z_cmd, 0);
        //  ROS_INFO("Target: %f", target_position_vector[2]);
        //  ROS_INFO("Current: %f", current_local_position.z);
        //  ROS_INFO("Effort: %f", z_pid_effort);
    }

    else
    {
        verti_control = 0;
        droneControlSignal(x_cmd, y_cmd, 0, 0);
    }

    if (xy_setpoint_dist < 0.5)
    {
        droneControlSignal(0, 0, 0, 0);
        ROS_INFO("Setpoint");
        waypoint_finished = true;
    }
}

void FlightPlanner::stepYaw()
{
    geometry_msgs::Vector3 position_offset;

    getLocalPositionOffset(position_offset, current_gps_location, start_gps_location);

    double x_offset_left = target_position_vector[0] - position_offset.x;
    double y_offset_left = target_position_vector[1] - position_offset.y;

    current_yaw_angle = toEulerAngle(current_drone_attitude).z;

    current_yaw_angle = (current_yaw_angle >= 0) ? current_yaw_angle : current_yaw_angle + 2 * C_PI;

    desired_yaw_angle = atan2(y_offset_left, x_offset_left);

    desired_yaw_angle = (desired_yaw_angle >= 0) ? desired_yaw_angle : desired_yaw_angle + 2 * C_PI;

    double yaw_diff = desired_yaw_angle - current_yaw_angle;

    // Yaw pid to be published to control signal.
    double yaw_pid = pid_yaw.update(desired_yaw_angle, current_yaw_angle, dt);

    double desired_yaw_angle_deg = RadToDeg(desired_yaw_angle);

    double current_yaw_deg = RadToDeg(current_yaw_angle);

    //ROS_INFO("Desired angle degree= %f", desired_yaw_angle_deg);

    // ROS_INFO("Current Yaw Angle %f", current_yaw_deg);

    // ROS_INFO("Yaw PID %f", yaw_pid);

    // Use Yaw angle flag for the control publisher
    droneControlSignal(0, 0, 0, yaw_pid, true, true);
    // Check if we are close to the required yaw.
    if (fabs(desired_yaw_angle_deg - current_yaw_deg) < 0.9)
    {
        yaw_flag = false;
        ROS_INFO("YAW");
    }
}

float FlightPlanner::setYaw(float yaw)
{
    target_yaw_angle = yaw;

    return yaw;
}

Eigen::Vector3d FlightPlanner::setTarget(float x, float y, float z)
{
    ROS_INFO("Target set as %f, %f, %f", x, y, z);
    target_position_vector << x, y, z;

    Eigen::Vector2d xy_pos_vec;
    xy_pos_vec << x, y;

    hori_target_norm = xy_pos_vec.norm();
    target_norm = target_position_vector.norm();

    return target_position_vector;
}

Eigen::Vector3d FlightPlanner::setHomeTarget(float x, float y, float z)
{
    //ROS_INFO("Home point set as %f, %f, %f", x, y, z);
    home_position_vector << x, y, z;

    home_target_norm = home_position_vector.norm();

    return home_position_vector;
}

Eigen::Vector3d FlightPlanner::getEffort(Eigen::Vector3d &target)
{
    double vector_length = target.norm();
    distance_to_setpoint = vector_length;

    // Initialise unit vector
    Eigen::Vector3d position_unit_vector = Eigen::Vector3d::Zero();
    // check for zero magnitude
    if (vector_length != 0)
    {
        // normalize vector;
        position_unit_vector = target.normalized();
    }

    return position_unit_vector;
}

Eigen::Vector3d FlightPlanner::getHomeEffort(Eigen::Vector3d &target)
{
    double vector_length = target.norm();
    home_distance = vector_length;

    // Initialise unit vector

    Eigen::Vector3d position_unit_vector = Eigen::Vector3d::Zero();

    // check for zero magnitude
    if (vector_length != 0)
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
    flightWaypoint.altitude = alt + altitude_offset; // add the UAV mission altitude offset from the ground to the drone's altitude

    ROS_INFO("Atitude offset for waypoint %d set as %f, drone altitude is now at %f", waypoint_index, altitude_offset, flightWaypoint.altitude);
    ROS_INFO("Latitude: %f, Longitude: %f", flightWaypoint.latitude, flightWaypoint.longitude);

    // Add UAV Waypoint to the list of waypoints to be visited
    appendFlightPlan(flightWaypoint, sampling_task, samplingTime);
}

bool FlightPlanner::isMissionFinished()
{
    // if waypoint count isn't zero and is equal to number of waypoint index,
    // we can safely assume the mission is finished..

    if (waypoint_index >= waypoint_count)
    {
        ROS_INFO("Mission Waypoint count: %d", waypoint_count);
        ROS_INFO("Mission Finished Waypoint index %d", waypoint_index);
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

    if (use_yaw_rate && use_ground_frame) // using yaw rate and ground frame
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

    ROS_INFO("Waypoint COORDINATES #%d : %f  %f  %f", waypoint_index + 1, flight_plan[waypoint_index].latitude, flight_plan[waypoint_index].longitude, flight_plan[waypoint_index].altitude);

    getLocalPositionOffset(offset_from_target, newWaypoint, start_gps_location);
    ROS_INFO("new Waypoint target offset  x: %f y: %f z: %f ", offset_from_target.x, offset_from_target.y, offset_from_target.z);

    // set UAV Target
    setTarget(offset_from_target.x, offset_from_target.y, offset_from_target.z);
}

bool FlightPlanner::reachedWaypoint()
{
    if (waypoint_finished)
    {
        ROS_INFO("We've reached waypoint %d", waypoint_index + 1);
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

    ROS_INFO("Landing Task %d", checkTask);
    ROS_INFO("Landing Task %f", sampleTime);
    ROS_INFO("Waypoint Lat: %f, Lon%f: ", GPS.front().latitude, GPS.front().longitude);
}

/*
Callbacks for the flight anomalies reported by the drone
Implement detailed safety routines here
Only Support A3/N3 Autopilots 
*/
void FlightPlanner::flightAnomalyCallback(const dji_sdk::FlightAnomaly::ConstPtr &msg)
{
    uint32_t flightAnomalydata = msg->data;

    if (flightAnomalydata)
    {
        // WHat should we do fo each use case?
        ROS_ERROR("Flight Anomaly detected by the UAV:");
        if (flightAnomalydata && dji_sdk::FlightAnomaly::IMPACT_IN_AIR)
        {
            ROS_ERROR("UAV Struck an objkect in air");
            // Define what should be done here
        }

        if (flightAnomalydata && dji_sdk::FlightAnomaly::RANDOM_FLY)
        {
            // Pause mission and let drone just stop in air.
            ROS_ERROR("UAV randomly implemented flight routine without command. Pausing Mission");
            droneControlSignal(0, 0, 0, 0, true, true);
        }

        if (flightAnomalydata && dji_sdk::FlightAnomaly::VERTICAL_CONTROL_FAIL)
        {
            ROS_ERROR("UAV has lost Height Control, Drone landing ");
            flightControl.land();
        }

        if (flightAnomalydata && dji_sdk::FlightAnomaly::HORIZONTAL_CONTROL_FAIL)
        {
            ROS_ERROR("UAV has lost roll/pitch control, Hovering in the air");
            droneControlSignal(0, 0, 0, 0, true, true);
        }

        if (flightAnomalydata && dji_sdk::FlightAnomaly::YAW_CONTROL_FAIL)
        {
            ROS_ERROR("UAV has lost yaw control");
            // do something here....
            // We don't essentially want to  stop anything if we can't yaw but
            // revisit this aspect later on I guess.
        }

        if (flightAnomalydata && dji_sdk::FlightAnomaly::AIRCRAFT_IS_FALLING)
        {
            ROS_ERROR("Aircraft is falling");
            // Well, we're fucked innit?
            // possibly try to stabilise?
            droneControlSignal(0, 0, 0, 0);
        }

        if (flightAnomalydata && dji_sdk::FlightAnomaly::STRONG_WIND_LEVEL1)
        {
            ROS_INFO("Strong Winds of LEVEL 1, Fly with caution");
        }

        if (flightAnomalydata && dji_sdk::FlightAnomaly::STRONG_WIND_LEVEL2)
        {
            ROS_INFO("Strong Winds of LEVEL 2, Drone flying back home");
            stepHome();
        }
        if (flightAnomalydata && dji_sdk::FlightAnomaly::COMPASS_INSTALLATION_ERROR)
        {
            ROS_ERROR("Compass Installed incorrectly, Aircraft will land");
            // Well, we're fucked innit?
            // possibly try to stabilise?
            flightControl.land();
        }

        if (flightAnomalydata && dji_sdk::FlightAnomaly::IMU_INSTALLATION_ERROR)
        {
            ROS_ERROR("IMU INSTALLATION ERROR, LANDING");
            flightControl.land();
        }

        if (flightAnomalydata && dji_sdk::FlightAnomaly::ESC_TEMPERATURE_HIGH)
        {
            ROS_INFO("ESC Temperature too high, keep an eye on this");
        }
        if (flightAnomalydata && dji_sdk::FlightAnomaly::ESC_DISCONNECTED)
        {
            ROS_ERROR("ESC DIsconnected, land drone now");
            flightControl.land();
        }
        if (flightAnomalydata && dji_sdk::FlightAnomaly::GPS_YAW_ERROR)
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

    if (!waypoint_lists.empty())
    {
        if (checkTask == 1)
        {
            bool performTask;

            if (checkM100())
            {
                performTask = flightControl.M100monitoredLanding();

                ROS_INFO_ONCE("Landing at the %dth waypoint", waypoint_index + 1);
            }

            else
            {
                performTask = flightControl.monitoredLanding();

                ROS_INFO_ONCE("Landing at the %dth waypoint", waypoint_index + 1);
            }

            if (performTask)
            {
                // Pause for a period of time here...
                // Integrate whatever task we're doing at this point.
                // TODO iintegrate UART_START_DMA Here.

                if (drone_version)
                {
                    ROS_INFO("Sleep time is %f", sampleTime);
                    ROS_INFO("UAV landed at  Lat: %f, Lon: %f", current_gps_location.latitude, current_gps_location.longitude);
                    ROS_INFO("Intended Landing area is: Lat: %f, Lon: %f", flight_plan[waypoint_index].latitude, flight_plan[waypoint_index].longitude);
                    getPositionError(current_gps_location, flight_plan[waypoint_index]);
                    ros::Duration(sampleTime).sleep();

                    if (waypoint_lists.size() > 1) // Only take off if we're not at the last waypoint and mission end is to autoland
                    {
                        verti_control = 1;
                        flightControl.M100monitoredTakeoff();
                    }

                    if (waypoint_lists.size() == 1 && checkMissionEnd == 3) // if we need to fly back home
                    {
                        verti_control = 1;
                        flightControl.M100monitoredTakeoff();
                    }
                }

                else
                {
                    ROS_INFO("in landing loop");
                    ROS_INFO("Sleep time is %f", sampleTime);
                    ros::Duration(sampleTime).sleep();
                    ROS_INFO("Drone Exited Sleeep");
                    verti_control = 1;
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
    // clear flight plan and reset waypoint information
    flight_plan.clear();
    waypoint_count = 0;
    waypoint_index = 0;
    //TODO UART_STOP_DMA
}
void FlightPlanner::getLocalPositionOffset(geometry_msgs::Vector3 &deltaENU, sensor_msgs::NavSatFix &target, sensor_msgs::NavSatFix &origin)
{
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;

    deltaENU.y = DegToRad(deltaLat) * C_EARTH;
    deltaENU.x = DegToRad(deltaLon) * C_EARTH * cos(DegToRad(target.latitude));
    //deltaENU.z = target.altitude - origin.altitude;
    deltaENU.z = target.altitude;
}

double FlightPlanner::getPositionError(sensor_msgs::NavSatFix &start_coord, sensor_msgs::NavSatFix &end_coord)
{
    double lat1_rad = DegToRad(start_coord.latitude);
    double lat2_rad = DegToRad(end_coord.latitude);
    double delta_lat_deg = end_coord.latitude - start_coord.latitude;
    double delta_lat = DegToRad(delta_lat_deg);
    double delta_lon_deg = end_coord.longitude - start_coord.longitude;
    double delta_lon = DegToRad(delta_lon_deg);
    double a = sin(delta_lat / 2) * sin(delta_lat / 2) + cos(lat1_rad) * cos(lat2_rad) * sin(delta_lon / 2) * sin(delta_lon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double d = C_EARTH * c;

    total_landing_error += d;

    ROS_INFO("Landing Error %f", d);
    std_msgs::Float32 error_msg;
    error_msg.data = d;

    error_publisher.publish(error_msg);

    return d;
}

void FlightPlanner::runMission()
{
    switch (state)
    {
    case MissionState::IDLE:
    {
        //TODO Fix waypoint mission from idle bug.
        //Probably just need to call setLocalPosition() again
        if (waypoint_count != 0)
        {

            waypoint_finished = false;
            yaw_flag = true;
            setWaypoint(flight_plan[waypoint_index]);

            ROS_INFO("MISSION START ROUTE FROM IDLE %d / %d ", waypoint_index + 1, waypoint_count);

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
        if (!reachedWaypoint())
        {
            if (yaw_flag)
            {
                stepYaw();
            }

            if (!yaw_flag)
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
        if (!isMissionFinished())
        {
            // if mission isn't finished, reset gps and local position to current position and continue mission.
            waypoint_finished = false;
            yaw_flag = true;
            start_gps_location = current_gps_location;
            start_local_position = current_local_position;
            setWaypoint(flight_plan[waypoint_index]);

            ROS_INFO("MISSION START ROUTE ARRIVED %d / %d ", waypoint_index + 1, waypoint_count);
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

        //ROS_INFO("Drone is case %d", checkMissionEnd);

        switch (checkMissionEnd)
        {
        case 1:
        {
            ROS_INFO("Hovering at position ");
            droneControlSignal(0, 0, 0, 0, true, true);
            break;
        }

        case 2:
        {
            state = MissionState::IDLE;
            break;
        }

        case 3:
        {

            // ROS_INFO("Computing home offsets");
            geometry_msgs::Vector3 offset_from_home;
            // ROS_INFO(" HOME Waypoint COORDINATES :  %f ,  %f, %f ", home_gps_location.latitude, home_gps_location.longitude, home_gps_location.altitude );
            getLocalPositionOffset(offset_from_home, home_gps_location, current_gps_location);
            //ROS_INFO("HOME Waypoint target offset  x: %f y: %f z: %f ", offset_from_home.x, offset_from_home.y, offset_from_home.z);

            setHomeTarget(offset_from_home.x, offset_from_home.y, offset_from_home.z);

            home_start_gps_location = current_gps_location;

            if (!home_reached)
            {
                stepHome();
            }

            //ROS_INFO("Home reached %d", home_reached);

            // Average landing error here:
            computeLandingError();

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

void FlightPlanner::computeLandingError()
{
    ROS_INFO("Number of waypoints is %d", waypoint_count);
    ROS_INFO("Total landing error is %f", total_landing_error);
    double avg_error = total_landing_error / waypoint_count;

    ROS_INFO("Average landing error for mission at %f m/s speed is %f", speedFactor, avg_error);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Flight_Control");

    FlightPlanner flight_planner;

    ROS_INFO("Running");

    ros::spin();

    return 0;
}
