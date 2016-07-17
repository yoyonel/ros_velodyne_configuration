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
SET( SRC_HTTPCLIENT
    servicelocator/src/boost_httpclient.cpp
    servicelocator/src/asynch_httpclient.cpp
    servicelocator/src/asynch_httpclient_imp.cpp
    servicelocator/src/synch_httpclient.cpp
    servicelocator/src/synch_httpclient_imp.cpp
    )
SET( SRC_VELODYNE_WEBSERVER
    servicelocator/src/vlp16webserver.cpp
    servicelocator/src/velodynewebserver.cpp
    servicelocator/src/velodynewebserver_meta.cpp
    servicelocator/src/vlp16webserver_ros.cpp
)
SET( SRC_PARSER_JSON
    servicelocator/src/parser_json.cpp
    )

#FILE(GLOB includeFiles RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
#    "${CMAKE_CURRENT_SOURCE_DIR}/servicelocator/include/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/servicelocator/include/*.instantiation")
#MESSAGE(includeFiles: ${includeFiles})
#MESSAGE(CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(servicelocator/include)
include_directories(../../../json_lib/src/)

##
#MESSAGE(Add flags for SRC_HTTPCLIENT)
#add_std_cxx11_flag(${SRC_HTTPCLIENT})
#MESSAGE(Add flags for SRC_VELODYNE_WEBSERVER)
#add_std_cxx11_flag(${SRC_VELODYNE_WEBSERVER})
#MESSAGE(Add flags for SRC_PARSER_JSON)
#add_std_cxx11_flag(${SRC_PARSER_JSON})
#MESSAGE(Add flags for includeFiles)
#add_std_cxx11_flag(${includeFiles})
##
#add_library(httpclient
#    ${SRC_HTTPCLIENT}
#)
##
#add_library(velodyne_webserver
#    ${SRC_VELODYNE_WEBSERVER}
#)
##
#add_library(parser_json
#    ${SRC_PARSER_JSON}
#)

ros_add_library(
    NAME    vlp16_servicelocator
    SRC     servicelocator/src/test_httpclient.cpp
            servicelocator/src/test_velodyne_webserver.cpp
            ${SRC_HTTPCLIENT} ${SRC_VELODYNE_WEBSERVER} ${SRC_PARSER_JSON}
    LIBS    velodyne_tools # httpclient velodyne_webserver parser_json
    DEPENDS velodyne_tools
    ADD_FLAG_FOR_CXX_11
)
