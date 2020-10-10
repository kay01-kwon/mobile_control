cmake_minimum_required(VERSION 2.8.3)
project(mobile_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  tf
)

add_message_files(
  FILES
  motorMsg.msg
#   Message2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mobile_control
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pd_control src/pd_control.cpp)
target_link_libraries(pd_control ${catkin_LIBRARIES})

add_executable(pd_control_linear src/pd_control_linear.cpp)
target_link_libraries(pd_control_linear ${catkin_LIBRARIES})


add_executable(wheel_odom src/wheel_odom.cpp)
target_link_libraries(wheel_odom ${catkin_LIBRARIES})

#add_executable(example_message_filters src/example_message_filters.cpp)
#target_link_libraries(example_message_filters ${catkin_LIBRARIES})
