cmake_minimum_required(VERSION 3.0.2)
project(car_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
  prius_msgs
)

find_package(gazebo REQUIRED)
find_package(ignition-msgs5 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros prius_msgs
  DEPENDS GAZEBO
)

###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
add_dependencies(PriusHybridPlugin ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS PriusHybridPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
