#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "m100_flight_planner/flight_base.h"

class FlightControl : public FlightBase
{
public:
    FlightControl();
    void takeOff();          //  un-monitored takeoff
    void land();             //
    void goHome();           // drone flies back home. This function uses the TASK_GO_HOME flag from the sdk.
    bool monitoredTakeoff(); //
    bool monitoredLanding();
    bool M100monitoredTakeoff();
    bool M100monitoredLanding();
    bool takeoffLand(int task);
    float computeTimeToLand(); // based on a 1 m/s descent speed

private:
    float timeToLand;
};

#endif // FLIGHT_CONTROL_H