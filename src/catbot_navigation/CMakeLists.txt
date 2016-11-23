cmake_minimum_required(VERSION 2.8.3)
project(catbot_navigation)

find_package(catkin REQUIRED roscpp std_msgs actionlib actionlib_msgs)


add_action_files(
  DIRECTORY action
  FILES GoToGoal.action
)


generate_messages( DEPENDENCIES actionlib_msgs actionlib std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES GoToGoalController
   CATKIN_DEPENDS actionlib_msgs
#  DEPENDS system_lib
)

SET(CMAKE_CXX_FLAGS "-std=gnu++11")

add_library(GoToGoalController src/GotoGoalController.cpp)
target_link_libraries(GoToGoalController ${catkin_LIBRARIES})
add_dependencies(GoToGoalController ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(catbotGoToGoal src/go-to-goal.cpp)
target_link_libraries(catbotGoToGoal GoToGoalController ${catkin_LIBRARIES})

add_executable(catbotGoToGoalClient src/go-to-goal-client.cpp)
target_link_libraries(catbotGoToGoalClient ${catkin_LIBRARIES})
