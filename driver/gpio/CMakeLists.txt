cmake_minimum_required(VERSION 2.8.3)
project(ros_gpio_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
)


roslint_cpp()
# roslint_add_test() # 编译报错



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_gpio_control
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/gpio_control_node.cpp
  src/gpio.cpp
)


add_executable(gpio_control_node 
               src/gpio_control_node.cpp
               src/jetsonGPIO.cpp
               include/jetsonGPIO.h)

target_link_libraries(gpio_control_node
  ${catkin_LIBRARIES}
  ros_gpio_control
)

