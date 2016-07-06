cmake_minimum_required(VERSION 2.8.3)
project(velodyne_configuration)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_SOURCE_DIR}/cmake")
MESSAGE(STATUS PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR})
include(cmake_tools)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    message_generation
    std_msgs
    dynamic_reconfigure
)
#
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  VLP16_StatusMessage.msg
  #
  VLP16_DiagnosticsRawMessage.msg
  VLP16_DiagnosticsMessage.msg
  #
  VLP16_SettingsMessage.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  VLP16_StatusService.srv
  #
  VLP16_DiagnosticsService.srv
  VLP16_DiagnosticsRawService.srv
  #
  VLP16_SettingsService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/VLP16_settings.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/tools)

add_subdirectory(src/settings)
add_subdirectory(src/status)
add_subdirectory(src/diagnostics)
add_subdirectory(src/webserver)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/server_status.launch
  launch/server_settings.launch
  launch/server_diagnostics.launch
  launch/servers_configuration.launch
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
