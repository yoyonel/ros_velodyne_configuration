###################################
## Velodyne Tools                ##
###################################
ros_add_library(
    NAME    velodyne_tools
    SRC     velodyne_tools.cc velodyne_tools_boost_asio.cc
    ADD_FLAG_FOR_CXX_11
)
