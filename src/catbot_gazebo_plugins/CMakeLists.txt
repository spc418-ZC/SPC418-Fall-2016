cmake_minimum_required(VERSION 2.8.3)
project(catbot_gazebo_plugins)

find_package(catkin REQUIRED
     gazebo_msgs
     roscpp
     std_msgs
     urdf
     tf
     tf2_ros
     geometry_msgs
     )
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(XML libxml-2.0)
    pkg_check_modules(OGRE OGRE)
    pkg_check_modules(OGRE-Terrain OGRE-Terrain)
    pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
    message(FATAL_ERROR "pkg-config is required; please install it")
endif()

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

execute_process(COMMAND
pkg-config --variable=plugindir OGRE
OUTPUT_VARIABLE OGRE_PLUGIN_PATH
OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

if (NOT GAZEBO_VERSION VERSION_LESS 6.0)
  catkin_package(  INCLUDE_DIRS include LIBRARIES gazebo_ros_elevator)
endif()

if (NOT GAZEBO_VERSION VERSION_LESS 7.3)
  catkin_package(INCLUDE_DIRS include LIBRARIES gazebo_ros_harness)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  gazebo_ros_utils
  CATKIN_DEPENDS roscpp gazebo_msgs std_msgs tf tf2_ros urdf nav_msgs geometry_msgs
  DEPENDS gazebo SDF
)
#add_library(gazebo_ros_utils src/gazebo_ros_utils/gazebo_ros_utils.cpp)
#target_link_libraries(gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_four_wheeled_drive src/four-wheeled-drive/gazebo_ros_four_wheeled_drive.cpp)
target_link_libraries(gazebo_ros_four_wheeled_drive gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS
    gazebo_ros_four_wheeled_drive
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
