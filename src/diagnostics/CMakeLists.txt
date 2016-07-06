###################################
## Velodyne diagnostics server   ##
###################################
ros_add_executable(
    NAME    velodyne_diagnostics_server
    SRC     diagnostics_server.cc
    LIBS    vlp16_webserver
    ADD_FLAG_FOR_CXX_11
)
