# dji_flight_planner

A ROS Package for flight planning on the DJI Matrice M100. 

## Prerequisites
- DJI M100
- An Android Device > V4.4
- ROS with [DJI Onboard SDK v3.80](https://github.com/dji-sdk/Onboard-SDK-ROS/tree/3.8)
- Tested with ROS 1 Kinetic and Melodic.

## Usage

 - Install the Android Application at this [link](https://github.com/ybabs/flightcontrol).
 - Connect your DJI C100 Controller.
 - run `roslaunch dji_sdk sdk.launch`
 - run `roslaunch dji_flight_planner m100_flight_planner`
 - profit
