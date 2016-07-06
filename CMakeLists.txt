project(velodyne_configuration)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_SOURCE_DIR}/../cmake")
#MESSAGE(STATUS PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR})
include(cmake_tools)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    message_generation
    std_msgs
    dynamic_reconfigure)
#
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

# ROS Messages
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

# ROS Dynamic Reconfigure Parameter
generate_dynamic_reconfigure_options(
  cfg/VLP16_settings.cfg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src/lib)

##############################
## Velodyne settings server ##
##############################
add_std_cxx11_flag("src/settings/settings_server.cc")
add_executable(velodyne_settings_server
    src/settings/settings_server.cc)
target_link_libraries(velodyne_settings_server
    velodyne_tools
    vlp16_settings
    ${catkin_LIBRARIES})
add_dependencies(velodyne_settings_server ${catkin_EXPORTED_TARGETS})

# install runtime files
install(TARGETS velodyne_settings_server
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     COMPONENT main
)
##############################

##############################
## Velodyne status server   ##
##############################
add_std_cxx11_flag("src/status/status_server.cc")

add_executable(velodyne_status_server
    src/status/status_server.cc)
target_link_libraries(velodyne_status_server
    velodyne_tools
#    velodyne_status
    ${catkin_LIBRARIES})
add_dependencies(velodyne_status_server ${catkin_EXPORTED_TARGETS})

# install runtime files
install(TARGETS velodyne_status_server
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     COMPONENT main
)
##############################


###################################
## Velodyne diagnostics server   ##
###################################
add_std_cxx11_flag("src/diagnostics/diagnostics_server.cc")
add_executable(velodyne_diagnostics_server
    src/diagnostics/diagnostics_server.cc)
target_link_libraries(velodyne_diagnostics_server
    velodyne_tools
#    velodyne_diagnostics
    ${catkin_LIBRARIES})
add_dependencies(velodyne_diagnostics_server ${catkin_EXPORTED_TARGETS})

# install runtime files
install(TARGETS velodyne_diagnostics_server
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     COMPONENT main
)
##############################
