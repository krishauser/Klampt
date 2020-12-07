# Finds ROS packages.
# If successful, sets the following variables:
#
# ROS_VERSION: string for the found ROS version (if not already set)
# ROS_INCLUDE_DIR: ROS header include path
# ROS_LIBRARIES: library files to link with in order to get ROS functionality
#
# If ROS_VERSION is not set, it will check ros versions jade, indigo, hydro, and groovy
# and configure variables for the newest version.  ROS_VERSION will also be set.
#
# If ROSDEPS is set, then it will add these dependencies to ROS_LIBRARIES
#


SET(AVAILABLE_ROS_VERSIONS "noetic;melodic;lunar;kinetic;jade;indigo;hydro;groovy")

IF(ROSDEPS)
ELSE(ROSDEPS)
  SET(ROSDEPS rosconsole roscpp roscpp_serialization rostime )
ENDIF(ROSDEPS)

SET(ROS_VERSION CACHE STRING "ROS version")
SET(ROS_FOUND FALSE)

IF(ROS_VERSION MATCHES "")
  MESSAGE("ROS_VERSION not defined")
  FOREACH(version ${AVAILABLE_ROS_VERSIONS})
    IF(NOT ROS_FOUND)
      IF(EXISTS /opt/ros/${version}/include/ros/ros.h)
        MESSAGE(STATUS "Found ros version ${version}")
        SET(ROS_VERSION2 ${version})
        SET(ROS_FOUND 1)
      ENDIF(EXISTS /opt/ros/${version}/include/ros/ros.h)
    ENDIF(NOT ROS_FOUND)
  ENDFOREACH(version)
ELSE(ROS_VERSION MATCHES "")
  MESSAGE("ROS_VERSION defined as " ${ROS_VERSION})
  IF(EXISTS /opt/ros/${version}/include/ros/ros.h)
    MESSAGE("Found ros version ${ROS_VERSION}")
    SET(ROS_FOUND 1)
    SET(ROS_VERSION2 ${version})
  ENDIF(EXISTS /opt/ros/${version}/include/ros/ros.h)
ENDIF(ROS_VERSION MATCHES "")
  
IF(NOT ROS_FOUND)
  MESSAGE(WARNING "ROS files not found")
ELSE(NOT ROS_FOUND)
  SET(ROS_PATH /opt/ros/${ROS_VERSION2})
  SET(ROS_INCLUDE_DIR ${ROS_PATH}/include)
  MESSAGE(STATUS "Detected ROS version ${ROS_VERSION2}")
  FOREACH(NAME ${ROSDEPS})
    FIND_LIBRARY(${NAME}_LIB NAMES ${NAME} PATHS ${ROS_PATH}/lib)
    LIST(APPEND ROS_LIBRARIES ${${NAME}_LIB})
  ENDFOREACH(NAME)
  MESSAGE(STATUS "ROS dependencies ${ROSDEPS} need the following libraries:")
  FOREACH(NAME ${ROS_LIBRARIES})
    MESSAGE(STATUS "  " ${NAME})
  ENDFOREACH(NAME)

  MESSAGE(STATUS "ROS will also need Boost libraries...")
  find_library (BOOST_SYSTEM_LIB "boost_system")
  LIST(APPEND ROS_LIBRARIES ${BOOST_SYSTEM_LIB})
ENDIF(NOT ROS_FOUND)
MARK_AS_ADVANCED(ROS_LIBRARIES)

