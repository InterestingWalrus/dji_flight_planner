#ifndef TEL_PUBLISHER_H
#define TEL_PUBLISHER_H

#include <ros/ros.h>
#include <tf/tf.h>

//DJI SDK Library
#include <djiosdk/dji_vehicle.hpp>
#include "dji_sdk/dji_sdk.h"

#include <dji_flight_planner/Compass.h>
#include <dji_flight_planner/GpsTimestamp.h>
#include <dji_flight_planner/GpsDetail.h>
#include <dji_flight_planner/FusedGps.h>
#include <dji_flight_planner/EscList.h>
#include <dji_flight_planner/Esc.h>



class DJITELEMETRY
{
   public: 

   DJITELEMETRY();
   void TelemetryBroadcast();
   void publishGpstimestamp();
   void publishGpsDetails();
   void publishFusedGps();
   void publishCompass();
   void publishEsc();   // Publishes ESC List also


    private:
    ros::Publisher  gps_timestamp_publisher;
    ros::Publisher gps_detail_publisher;
    ros::Publisher fused_gps_publisher;
    ros::Publisher esc_publisher;
    ros::Publisher compass_publisher;
    ros::Publisher esc_list_publisher;

    ros::NodeHandle n;

    

};






#endif