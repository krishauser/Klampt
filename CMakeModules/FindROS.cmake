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


SET(AVAILABLE_ROS_VERSIONS "jade;indigo;hydro;groovy")

IF(ROSDEPS)
ELSE(ROSDEPS)
  SET(ROSDEPS rosconsole roscpp roscpp_serialization rostime )
ENDIF(ROSDEPS)

SET(ROS_VERSION CACHE STRING "ROS version")
SET(ROS_FOUND FALSE)

IF(NOT DEFINED ROS_VERSION)
  MESSAGE("ROS_VERSION not defined")
  FOREACH(version ${AVAILABLE_ROS_VERSIONS})
    IF(NOT DEFINED ROS_VERSION)
      FIND_PATH(ROS_H ros.h PATHS /opt/ros/${version}/include/ros)
      IF(ROS_H)
        MESSAGE(STATUS "Found ros version ${version}")
        SET(ROS_VERSION ${version} CACHE STRING "ROS version")
        SET(ROS_FOUND 1)
      ENDIF(ROS_H)
    ENDIF(NOT DEFINED ROS_VERSION)
  ENDFOREACH(version)
ELSE(NOT DEFINED ROS_VERSION)
  MESSAGE("ROS_VERSION defined as " ${ROS_VERSION})
  FIND_PATH(ROS_H ros.h PATHS /opt/ros/${ROS_VERSION}/include/ros)
  IF(ROS_H)
    MESSAGE("Found ros version ${ROS_VERSION}")
    SET(ROS_FOUND 1)
  ENDIF(ROS_H)
ENDIF(NOT DEFINED ROS_VERSION)
  
IF(NOT ROS_FOUND)
  MESSAGE(WARNING "ROS files not found")
ELSE(NOT ROS_FOUND)
  SET(ROS_PATH /opt/ros/${ROS_VERSION})
  SET(ROS_INCLUDE_DIR ${ROS_PATH}/include)
  MESSAGE(STATUS "Detected ROS version ${ROS_VERSION}")
  FOREACH(NAME ${ROSDEPS})
    FIND_LIBRARY(${NAME}_LIB NAMES ${NAME} PATHS ${ROS_PATH}/lib)
    LIST(APPEND ROS_LIBRARIES ${${NAME}_LIB})
  ENDFOREACH(NAME)
  MESSAGE(STATUS "ROS dependencies ${ROSDEPS} need the following libraries:")
  FOREACH(NAME ${ROS_LIBRARIES})
    MESSAGE(STATUS "  " ${NAME})
  ENDFOREACH(NAME)
ENDIF(NOT ROS_FOUND)
MARK_AS_ADVANCED(ROS_LIBRARIES)

