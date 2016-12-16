cmake_minimum_required(VERSION 2.8.3)
project(assignment_3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)


SET(CMAKE_CXX_FLAGS "-std=gnu++11")

catkin_package (
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(input_controller src/input-controller.cpp)
target_link_libraries(input_controller
   ${catkin_LIBRARIES}
)
