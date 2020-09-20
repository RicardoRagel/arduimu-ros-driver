cmake_minimum_required(VERSION 2.8.3)
project(arduimu_ros_driver)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arduimu_ros_driver_lib
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(arduimu_rpy src/arduimu_rpy.cpp)
target_link_libraries(arduimu_rpy ${catkin_LIBRARIES})

add_executable(arduimu_sensors src/arduimu_sensors.cpp)
target_link_libraries(arduimu_sensors ${catkin_LIBRARIES})