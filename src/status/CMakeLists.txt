##############################
## Velodyne status server   ##
##############################
ros_add_executable(
    NAME    velodyne_status_server
    SRC     status_server.cc
    LIBS    vlp16_webserver vlp16_servicelocator
    ADD_FLAG_FOR_CXX_11
)
