cmake_minimum_required(VERSION 2.8.3)
project(assignment_2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  message_runtime
  tf
)

SET(CMAKE_CXX_FLAGS "-std=gnu++11")

add_message_files(FILES Pose2D.msg)

generate_messages(
    DEPENDENCIES
    std_msgs
)

include_directories( include ${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assginment-2
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(imu_node src/imu-node.cpp)
target_link_libraries(imu_node ${catkin_LIBRARIES})
add_dependencies(imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odometry_node src/odometry-node.cpp)
target_link_libraries(odometry_node ${catkin_LIBRARIES})

add_executable(laser_scan_node src/laser-scan-node.cpp)
target_link_libraries(laser_scan_node ${catkin_LIBRARIES})
