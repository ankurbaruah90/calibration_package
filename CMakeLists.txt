cmake_minimum_required(VERSION 2.8.3)
project(calibration_package)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  laser_geometry
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  pcl_ros
  pcl_conversions
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -std=c++11")


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES calibration_package
  CATKIN_DEPENDS cv_bridge image_geometry image_transport laser_geometry roscpp rospy sensor_msgs std_msgs tf tf2 tf2_ros tf2_msgs tf2_sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(line_node src/line.cpp)
target_link_libraries(line_node ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(pose src/pose.cpp)
target_link_libraries(pose ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(transform src/transform.cpp)
target_link_libraries(transform ${OpenCV_LIBS} ${catkin_LIBRARIES} gsl gslcblas)

