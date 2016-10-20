# Require a minimum version of the CMake build configuration
cmake_minimum_required(VERSION 2.8.3)

project(catbot_control)

# Linking catkin dependency Packages to the project
find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs nav_msgs tf
)

# Configuring the package to resolve any package dependencies of packages that depend on this package
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES catbot_control
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs tf
#  DEPENDS system_lib
)

# add the include directories present in catkin_INCLUDE_DIRS macro to the package
# any other include directories that are used in the c++ files must be listed in here
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_node src/teleop/catbot_teleop.cpp)
add_executable(kinematic_model src/kinematics/kinematic_model.cpp)
add_executable(encoder src/diff_catbot_encoder.cpp)
add_executable(fourWD_controller src/4WD_control.cpp)

# Link libraries to executables
target_link_libraries(teleop_node ${catkin_LIBRARIES})
target_link_libraries(kinematic_model ${catkin_LIBRARIES})
target_link_libraries(encoder ${catkin_LIBRARIES})
target_link_libraries(fourWD_controller ${catkin_LIBRARIES})
