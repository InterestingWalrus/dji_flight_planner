#include "m100_flight_planner/telemetry_publisher.h"

DJITELEMETRY::DJITELEMETRY()
{
  
  compass_publisher = n.advertise<dji_flight_planner::Compass>("/dji_sdk/mag", 10);
  gps_timestamp_publisher = n.advertise<dji_flight_planner::GpsTimestamp>("/dji_sdk/GpsTimestamp", 10);
  gps_detail_publisher = n.advertise<dji_flight_planner::GpsDetail>("/dji_sdk/gpsDetails",10);
  esc_publisher = n.advertise<dji_flight_planner::Esc>("/dji_sdk/escDetails",10);
  esc_list_publisher = n.advertise<dji_flight_planner::EscList>("/dji_sdk/EscList", 10);
  fused_gps_publisher = n.advertise<dji_flight_planner::FusedGps>("/dji_sdk/FusedGps", 10);

}


void DJITELEMETRY::publishCompass()
{
    ros::Time time_now = ros::Time::now();

    dji_flight_planner::Compass msg;
    msg.header.stamp = time_now;
    msg.header.frame_id = "body_FLU"; // in the FLU frame.

    DJI::OSDK::Telemetry::Mag  FusedMag;

    msg.x = FusedMag.x;
    msg.y = FusedMag.y;
    msg.z = FusedMag.z;

    compass_publisher.publish(msg);
   
}

void DJITELEMETRY::publishEsc()
{
    int MAX_ESC = 8;
    ros::Time time_now = ros::Time::now();
    
    
    dji_flight_planner::Esc msg;
    dji_flight_planner::EscList Esc_vec;
    DJI::OSDK::Telemetry::EscData esc_data;
    msg.header.stamp = time_now;
    msg.header.frame_id = "body_FLU";
    DJI::OSDK::Telemetry::ESCStatusIndividual escDetails;
    

    msg.current = escDetails.current;    
    msg.speed = escDetails.speed;
    msg.voltage = escDetails.voltage;
    msg.temperature = escDetails.temperature;
    msg.stall  = escDetails.stall;
    msg.empty = escDetails.empty;
    msg.unbalanced = escDetails.unbalanced;
    msg.escDisconnected = escDetails.escDisconnected;
    msg.temperatureHigh = escDetails.temperatureHigh;

    esc_publisher.publish(msg);

    for(int i = 0; i < MAX_ESC; i ++)
    {
        msg.speed = esc_data.esc[i].speed;
        msg.current = esc_data.esc[i].current;    
        msg.voltage = esc_data.esc[i].voltage;
        msg.temperature = esc_data.esc[i].temperature;
        msg.stall  = esc_data.esc[i].stall;
        msg.empty = esc_data.esc[i].empty;
        msg.unbalanced = esc_data.esc[i].unbalanced;
        msg.escDisconnected = esc_data.esc[i].escDisconnected;
        msg.temperatureHigh = esc_data.esc[i].temperatureHigh;

          

        Esc_vec.Esc_list.push_back(msg);
    }

    esc_list_publisher.publish(Esc_vec);

}

void DJITELEMETRY::publishFusedGps()
{

    ros::Time time_now = ros::Time::now();

    dji_flight_planner::FusedGps msg;
    msg.header.stamp = time_now;
    msg.header.frame_id = "map"; // World frame

    DJI::OSDK::Telemetry::GPSFused fusedGPS;

    msg.latitude = fusedGPS.latitude;
    msg.longitude = fusedGPS.longitude;
    msg.altitude = fusedGPS.altitude;
    msg.visibleSatelliteNumber = fusedGPS.visibleSatelliteNumber;

    fused_gps_publisher.publish(msg);
}

void DJITELEMETRY::publishGpsDetails()
{
    ros::Time time_now = ros::Time::now();

    dji_flight_planner::GpsDetail msg;
    msg.header.stamp = time_now;
    msg.header.frame_id = "map"; // World frame


    DJI::OSDK::Telemetry::GPSDetail gpsdetails;

    msg.fix = gpsdetails.fix;
    msg.hdop = gpsdetails.hdop;
    msg.pdop = gpsdetails.pdop;
    msg.gnssStatus = gpsdetails.gnssStatus;
    msg.hacc = gpsdetails.hacc;
    msg.sacc = gpsdetails.sacc;
    msg.usedGPS = gpsdetails.usedGPS;
    msg.usedGLN = gpsdetails.usedGLN;
    msg.NSV = gpsdetails.NSV;
    msg.GPScounter = gpsdetails.GPScounter;

    fused_gps_publisher.publish(msg);


}

void DJITELEMETRY::publishGpstimestamp()
{
    ros::Time time_now = ros::Time::now();
    dji_flight_planner::GpsTimestamp msg;

    msg.header.stamp = time_now;
    msg.header.frame_id = "map"; // World frame


    DJI::OSDK::Telemetry::PositionTimeStamp fixTimeStamp;

    msg.gpsDate = fixTimeStamp.date;
    msg.gpsTime = fixTimeStamp.time;

    gps_timestamp_publisher.publish(msg);

}
