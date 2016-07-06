##############################
## Velodyne settings server ##
##############################
ros_add_executable(
    NAME    velodyne_settings_server
    SRC     settings_server.cc
    LIBS    vlp16_webserver vlp16_settings
    ADD_FLAG_FOR_CXX_11
)
