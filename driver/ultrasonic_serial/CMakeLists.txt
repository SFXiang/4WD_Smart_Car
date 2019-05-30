
cmake_minimum_required(VERSION 2.8.3)
project(ultrasonic_serial)
 find_package(catkin REQUIRED COMPONENTS
    roscpp
    serial
    std_msgs
    can_msgs
  )
 
 catkin_package(
   CATKIN_DEPENDS
     serial
     std_msgs
     can_msgs
 )
 include_directories(
   ${catkin_INCLUDE_DIRS}
 )
 add_executable(ultrasonic_serial src/ultrasonic_serial_node.cpp)
 target_link_libraries(ultrasonic_serial
   ${catkin_LIBRARIES}
 )


