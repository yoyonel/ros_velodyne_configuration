add_std_cxx11_flag("velodyne_tools.cc")
add_std_cxx11_flag("velodyne_tools_boost_asio.cc")
#add_std_cxx11_flag("velodyne_webserver.cc")
add_std_cxx11_flag("vlp16_webserver.cc")
add_std_cxx11_flag("vlp16_webserver_services.cc")
#add_std_cxx11_flag("velodyne_diagnostics.cpp")
add_std_cxx11_flag("vlp16_settings.cpp")
#
add_std_cxx11_flag("../../include/velodyne_tools.h")


add_library(velodyne_tools
    velodyne_tools.cc
    velodyne_tools_boost_asio.cc
    vlp16_webserver.cc
    vlp16_webserver_services.cc
)
target_link_libraries(velodyne_tools ${catkin_LIBRARIES})

add_library(vlp16_settings vlp16_settings.cpp)
target_link_libraries(vlp16_settings ${catkin_LIBRARIES})

#add_library(velodyne_status velodyne_status.cpp)
#target_link_libraries(velodyne_status ${catkin_LIBRARIES})

#add_library(velodyne_diagnostics velodyne_diagnostics.cpp)
#target_link_libraries(velodyne_diagnostics ${catkin_LIBRARIES})

# ####################
# INSTALL
# ####################
install(TARGETS velodyne_tools
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS vlp16_settings
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(TARGETS velodyne_status
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(TARGETS velodyne_diagnostics
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
