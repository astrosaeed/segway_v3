cmake_minimum_required(VERSION 2.8.3)
project(segway_assisted_teleop)

find_package(cmake_modules REQUIRED)

set(THIS_PACKAGE_ROS_DEPS
  actionlib
  angles
  base_local_planner
  costmap_2d
  filters
  geometry_msgs
  message_filters
  move_base_msgs
  pluginlib
  roscpp
  roslib
  sensor_msgs
  tf
)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})

# Starting in ROS Jade, using cmake_modules to find Eigen is deprecated.
# Instead, find_package(Eigen3) is called which uses the cmake config included
# with the system install of Eigen. For compatibility with older ROS distros,
# we fall back to the cmake_modules version if needed.
#
# See http://wiki.ros.org/jade/Migration#Eigen_CMake_Module_in_cmake_modules
#
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules (already found above)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${EIGEN3_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
)

add_executable(segway_assisted_teleop src/segway_assisted_teleop.cpp)
target_link_libraries(segway_assisted_teleop trajectory_planner_ros ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS segway_assisted_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


