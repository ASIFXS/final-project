cmake_minimum_required(VERSION 3.0.2)
project(ur_agent)

## Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  controller_manager_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  geometry_msgs
)

## Declare catkin package
catkin_package()

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(
  DIRECTORY 
    scripts/
    scripts/tools/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Python scripts and tools
install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## If you want tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Add your tests here
endif()
