#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "m100_flight_planner/flight_base.h"

class FlightControl : public FlightBase
{
    public:
        FlightControl();
        void activate(); // Activate the drone for SDK control
        void takeOff(); //  un-monitored takeoff
        void land(); //
        void goHome(); // drone flies back home. This function uses the TASK_GO_HOME flag from the sdk. 
        bool monitoredTakeoff();  // 
        bool monitoredLanding();
        bool obtainControl();
        bool releaseControl();
        bool check_M100();
        bool M100monitoredTakeoff();
        bool M100monitoredLanding();
        bool takeoff_land(int task);
        float computeTimeToLand(); // based on a 1 m/s descent speed
       

    private:

    uint16_t satellite_Strength;
    uint8_t gps_health;
    float height_above_takeoff;
    float timeToLand;
};

#endif // FLIGHT_CONTROL_H