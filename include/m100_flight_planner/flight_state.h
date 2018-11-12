#ifndef DRONE_STATE
#define DRONE_STATE
enum class DroneState
{

    STATE_IDLE = 0, // Drone in the ground
    STATE_LANDING = 1, // Drone descending
    STATE_TAKING_OFF = 2,  // Drone taking off
    STATE_AT_WAYPOINT = 3, // Drone at goal
    STATE_IN_MISSION = 4, // During mission
    STATE_ARRRIVING_AT_WAYPOINT = 5




};

#endif