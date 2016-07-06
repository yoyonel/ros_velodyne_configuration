##############################
## Webserver                ##
##############################


##############################
## Webserver - VLP16        ##
##############################
ros_add_library(
    NAME    vlp16_webserver
    SRC     vlp16_webserver.cc
    LIBS    velodyne_tools
    DEPENDS velodyne_tools
    ADD_FLAG_FOR_CXX_11
)

##################################
## Webserver - VLP16 - Settings ##
##################################
ros_add_library(
    NAME    vlp16_settings
    SRC     vlp16_settings.cpp
    LIBS    velodyne_tools
    DEPENDS velodyne_tools
    ADD_FLAG_FOR_CXX_11
)
