cmake_minimum_required(VERSION 2.8.3)
project(catbot_localization)

find_package(catkin REQUIRED roscpp std_msgs nav_msgs sensor_msgs tf catbot_control)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES catbot_localization
   CATKIN_DEPENDS catbot_control
#  DEPENDS system_lib
)

SET(CMAKE_CXX_FLAGS "-std=gnu++11")
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${catbot_control_INCLUDE_DIRS}
)

add_executable(ground_truth src/ground-truth/ground_truth.cpp)
add_executable(ground_truth_path_generator src/ground-truth-path-generator.cpp)
add_executable(ground_truth_pose_generator src/ground-truth-pose-generator.cpp)
add_executable(odometry_pose_generator src/odometry-pose-generator.cpp)
add_executable(odometry_path_generator src/odometry-path-generator.cpp)
add_executable(vodometry_path_generator src/visual-odometry-path-generator.cpp)

target_link_libraries(ground_truth ${catkin_LIBRARIES})
target_link_libraries(ground_truth_path_generator ${catkin_LIBRARIES})
target_link_libraries(ground_truth_pose_generator ${catkin_LIBRARIES})
target_link_libraries(odometry_pose_generator     ${catkin_LIBRARIES})
target_link_libraries(odometry_path_generator     ${catkin_LIBRARIES})
target_link_libraries(vodometry_path_generator   ${catkin_LIBRARIES})
