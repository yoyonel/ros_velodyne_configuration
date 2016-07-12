##############################
## Webserver                ##
##############################


##############################
## Webserver - VLP16        ##
##############################
#
SET( SRC_WEBSERVER_SERVICES
    services/vlp16_webserver_services_settings.cpp
    services/vlp16_webserver_services_status.cpp
    services/vlp16_webserver_services_diagnostics.cpp
    services/vlp16_webserver_services.cpp
)

SET( SRC_WEBSERVER_CONNECTIONS
    connections/vlp16_webserver_curl.cpp
    connections/vlp16_webserver_boost_asio_synch.cpp
    connections/vlp16_webserver_boost_asio_asynch.cpp
    boost_asio/velodyne_tools_boost_asio.cc
    boost_asio/velodyne_tools_boost_asio_clientsynch.cc
    boost_asio/velodyne_tools_boost_asio_clientasynch.cc
)

ros_add_library(
    NAME    vlp16_webserver
    SRC     vlp16_webserver.cc
            ${SRC_WEBSERVER_SERVICES}
            ${SRC_WEBSERVER_CONNECTIONS}
# A terme a voir pour retirer cette dépendance de velodyne_tools
# pas forcement utile ...
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


##################################
## Test: Service Locator
##################################
SET( SRC_SERVICEPROVIDERS
    servicelocator/src/asynch_httpclient.cpp
    servicelocator/src/asynch_httpclient_imp.cpp
    servicelocator/src/synch_httpclient.cpp
    servicelocator/src/synch_httpclient_imp.cpp
)
include_directories(servicelocator/include)

ros_add_library(
    NAME    vlp16_servicelocator
    SRC     servicelocator/src/test_httpclient.cpp
            ${SRC_SERVICEPROVIDERS}
    LIBS    velodyne_tools
    DEPENDS velodyne_tools
    ADD_FLAG_FOR_CXX_11
)
