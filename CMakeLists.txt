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
  tf2_ros
  pcl_ros
  pcl_conversions
  message_generation
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -std=c++11")

add_message_files(
   FILES
   Particle.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES calibration_package
  CATKIN_DEPENDS cv_bridge image_geometry image_transport laser_geometry
  roscpp rospy sensor_msgs std_msgs tf tf2_ros geometry_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(line_node src/line.cpp)
target_link_libraries(line_node ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(checkerboard_pose src/pose.cpp)
target_link_libraries(checkerboard_pose ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(calibrate src/transform.cpp)
add_dependencies(calibrate ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(calibrate ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(optimize src/pso.cpp)
add_dependencies(optimize ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(optimize ${OpenCV_LIBS} ${catkin_LIBRARIES} gsl gslcblas)

