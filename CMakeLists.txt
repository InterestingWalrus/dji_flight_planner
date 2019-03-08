cmake_minimum_required(VERSION 2.8.3)
project(dji_flight_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dji_sdk
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
)


find_package(DJIOSDK REQUIRED)
find_package(Eigen3 REQUIRED)


add_message_files(
  FILES
  Compass.msg
  Esc.msg
  EscList.msg
  FusedGps.msg
  GpsDetail.msg
  GpsTimestamp.msg
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
  CATKIN_DEPENDS
        message_runtime    
        geometry_msgs
        nav_msgs
        std_msgs

)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include ${EIGEN3_INCLUDE_DIR}
)



add_executable(m100_flight_planner src/flight_control.cpp src/flight_planner.cpp src/mobile_comm.cpp src/PID.cpp src/telemetry_publisher.cpp )


target_link_libraries(m100_flight_planner
		${catkin_LIBRARIES}
		${EIGEN_LIBRARIES}
    ${DJIOSDK_LIBRARIES}
		)

add_dependencies(m100_flight_planner dji_sdk_generate_messages_cpp dji_flight_planner_generate_messages_cpp)

target_link_libraries(m100_flight_planner
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
