function(ROS_ADD)
  # url: https://cmake.org/cmake/help/v3.0/module/CMakeParseArguments.html
  set(options
      IS_EXECUTABLE
      IS_LIBRARY
      ADD_FLAG_FOR_CXX_11
      ADD_FLAG_FOR_CXX_14
      ADD_ROS_DEPENDS
      ADD_ROS_CATKINS_LIBS
      USE_ROS_INSTALLATION
      PRINT_DEBUG)
  set(oneValueArgs NAME)
  set(multiValueArgs SRC LIBS DEPENDS)
  cmake_parse_arguments(ROS_ADD "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
  ########################################################

  IF(${ROS_ADD_ADD_FLAG_FOR_CXX_11} MATCHES "TRUE")
    add_std_cxx11_flag("${ROS_ADD_SRC}")
  ENDIF()

  IF(${ROS_ADD_ADD_FLAG_FOR_CXX_14} MATCHES "TRUE")
    add_std_cxx14_flag("${ROS_ADD_SRC}")
  ENDIF()

  ########################################################

  IF(${ROS_ADD_IS_EXECUTABLE} MATCHES "TRUE")
    add_executable(${ROS_ADD_NAME} ${ROS_ADD_SRC})
  ENDIF()

  IF(${ROS_ADD_IS_LIBRARY} MATCHES "TRUE")
    add_library(${ROS_ADD_NAME} ${ROS_ADD_SRC})
  ENDIF()

  ########################################################

  IF(${ROS_ADD_ADD_ROS_DEPENDS} MATCHES "TRUE")
    LIST(APPEND
        ROS_ADD_DEPENDS
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )
  ENDIF()
  IF(ROS_ADD_DEPENDS)
    ## Add cmake target DEPENDS of the executable
    ## same as for the library above
    add_dependencies(${ROS_ADD_NAME} ${ROS_ADD_DEPENDS})
  ENDIF()

  ########################################################

  IF(${ROS_ADD_ADD_ROS_CATKINS_LIBS} MATCHES "TRUE")
    ## Specify libraries to link a library or executable target against
    LIST(APPEND ROS_ADD_LIBS ${catkin_LIBRARIES})
  ENDIF()
  IF(ROS_ADD_LIBS)
    ## Specify libraries to link a library or executable target against
    target_link_libraries(${ROS_ADD_NAME} ${ROS_ADD_LIBS})
  ENDIF()

  ########################################################

  IF(${ROS_ADD_USE_ROS_INSTALLATION} MATCHES "TRUE")
    ## Mark executables and/or libraries for installation
    install(TARGETS ${ROS_ADD_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
  ENDIF()

  ########################################################
  IF(${ROS_ADD_PRINT_DEBUG} MATCHES "TRUE")
    foreach(option ${options})
      MESSAGE("- ${option} = ${ROS_ADD_${option}}")
    endforeach()
    MESSAGE("* NAME = ${ROS_ADD_NAME}")
    MESSAGE("# SRC = ${ROS_ADD_SRC}")
    MESSAGE("# LIBS = ${ROS_ADD_LIBS}")
    MESSAGE("# DEPENDS = ${ROS_ADD_DEPENDS}")
    #
    MESSAGE("? IS_EXECUTABLE = ${ROS_ADD_IS_EXECUTABLE}")
    MESSAGE("? IS_LIBRARY = ${ROS_ADD_IS_LIBRARY}")
    #
    MESSAGE("? USE_ROS_INSTALLATION = ${ROS_ADD_USE_ROS_INSTALLATION}")
    MESSAGE("? ADD_ROS_CATKINS_LIBS = ${ROS_ADD_ADD_ROS_CATKINS_LIBS}")
    MESSAGE("? ADD_ROS_DEPENDS = ${ROS_ADD_ADD_ROS_DEPENDS}")

  ENDIF()

endfunction()

SET(FLAGS_FOR_ROS
    USE_ROS_INSTALLATION
    ADD_ROS_CATKINS_LIBS
    ADD_ROS_DEPENDS
)

function(ROS_ADD_EXECUTABLE)
  ros_add(
    IS_EXECUTABLE
    ${FLAGS_FOR_ROS}
    ${ARGN}
  )
endfunction(ROS_ADD_EXECUTABLE)

function(ROS_ADD_LIBRARY)
  ros_add(
    IS_LIBRARY
    ${FLAGS_FOR_ROS}
    ${ARGN}
)
endfunction(ROS_ADD_LIBRARY)

function(ROS_ADD_EXECUTABLES NAMES_SAMPLES FLAGS_FOR_SAMPLES)
  foreach(name_sample ${NAMES_SAMPLES})
    ros_add_executable(
      NAME ${name_sample}
      SRC ${name_sample}.cpp
      ${FLAGS_FOR_SAMPLES}
      ${FLAGS_FOR_ROS}
    )
  endforeach()
endfunction()

function(ROS_ADD_LIBRARIES NAMES_SAMPLES FLAGS_FOR_SAMPLES)
  foreach(name_sample ${NAMES_SAMPLES})
    ros_add_library(
      NAME ${name_sample}
      SRC ${name_sample}.cpp
      ${FLAGS_FOR_SAMPLES}
      ${FLAGS_FOR_ROS}
    )
  endforeach()
endfunction()
