cmake_minimum_required(VERSION 2.8.3)
project(catbot_localization)

find_package(catkin REQUIRED roscpp std_msgs nav_msgs sensor_msgs tf catbot_control message_generation message_runtime)


add_service_files(FILES getIMU.srv)

generate_messages(
    DEPENDENCIES  std_msgs geometry_msgs
)

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
add_executable(imu_server_node	src/kalman-filter/imu-server-node.cpp)
add_executable(kalman_filter_node src/kalman-filter/kalman-filter-node.cpp)
add_library(KalmanFilter src/kalman-filter/kalman-filter.cpp)

add_dependencies(KalmanFilter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imu_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ground_truth ${catkin_LIBRARIES})
target_link_libraries(ground_truth_path_generator ${catkin_LIBRARIES})
target_link_libraries(ground_truth_pose_generator ${catkin_LIBRARIES})
target_link_libraries(odometry_pose_generator     ${catkin_LIBRARIES})
target_link_libraries(odometry_path_generator     ${catkin_LIBRARIES})
target_link_libraries(vodometry_path_generator   ${catkin_LIBRARIES})
target_link_libraries(imu_server_node   ${catkin_LIBRARIES})
target_link_libraries(kalman_filter_node  KalmanFilter ${catkin_LIBRARIES})
target_link_libraries(KalmanFilter ${catkin_LIBRARIES})
