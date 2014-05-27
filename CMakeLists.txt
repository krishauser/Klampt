CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
PROJECT( Klampt )

# version (e.g. for packaging)
set(KLAMPT_MAJOR_VERSION 0)
set(KLAMPT_MINOR_VERSION 6)
set(KLAMPT_PATCH_VERSION 0)
set(KLAMPT_VERSION ${KLAMPT_MAJOR_VERSION}.${KLAMPT_MINOR_VERSION}.${KLAMPT_PATCH_VERSION})
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR} )

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}")
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

LINK_DIRECTORIES(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# define the Klamp't library subdirectories in the SUBDIRS variable
SET(SUBDIRS Contact Control IO Interface Modeling Planning Simulation View)

# Get all compilation files in subdirectories
foreach(s ${SUBDIRS})
  file(GLOB SUBDIR_SRCS  ${PROJECT_SOURCE_DIR}/${s}/*.cpp )
  SET(Klampt_SRCS ${Klampt_SRCS} ${SUBDIR_SRCS})
  SET(Klampt_${s}_SRCS ${SUBDIR_SRCS})
endforeach(s)

#add all compilation files to the Klamp't library
ADD_LIBRARY( Klampt STATIC ${Klampt_SRCS})




##### External libraries: KrisLibrary, its dependencies, and ODE #####

SET(KLAMPT_ROOT ${PROJECT_SOURCE_DIR})
INCLUDE(KlamptDependencies)
#defines KLAMPT_INCLUDE_DIRS, KLAMPT_DEFINITIONS, KLAMPT_LIBRARIES
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})

#show in CMake cache
SET(KLAMPT_INCLUDE_DIRS ${KLAMPT_INCLUDE_DIRS} CACHE STRING "Klamp't include directories" FORCE)
SET(KLAMPT_DEFINITIONS ${KLAMPT_DEFINITIONS} CACHE STRING "Klamp't compiler definitions" FORCE)
SET(KLAMPT_LIBRARIES ${KLAMPT_LIBRARIES} CACHE STRING "Klamp't link libraries" FORCE)

#add Klamp't onto KLAMPT_LIBRARIES
SET(KLAMPT_LIBRARIES Klampt ${KLAMPT_LIBRARIES})
SET(KLAMPT_INCLUDE_DIRS ${KLAMPT_INCLUDE_DIRS} ${KLAMPT_ROOT})


####### Klamp't Applications #########
IF(NOT USE_QT4_APPS)
  #USE_QT4_APPS not set
  FIND_PACKAGE(Qt4)
  IF(QT4_FOUND)
    SET(USE_QT4_APPS TRUE)
  ELSE(QT4_FOUND)
    SET(USE_QT4_APPS FALSE)
  ENDIF(QT4_FOUND)
ENDIF(NOT USE_QT4_APPS)
ADD_SUBDIRECTORY(Main)

####### Klamp't Examples #########
ADD_SUBDIRECTORY(Examples)

####### Klamp't Documentation #########
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.conf.in ${CMAKE_CURRENT_BINARY_DIR}/doxygen.conf @ONLY)
  ADD_CUSTOM_TARGET(docs ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxygen.conf
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating documentation (Doxygen) at location ${PROJECT_SOURCE_DIR}/doc/html/")
ENDIF(DOXYGEN_FOUND)

# Python API directory
ADD_SUBDIRECTORY(Python)
ADD_DEPENDENCIES(python Klampt)



# uninstall target
configure_file(
    "${CMAKE_MODULE_PATH}/CMakeUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)


#install targets
install(TARGETS Klampt
  ARCHIVE
  DESTINATION lib
  COMPONENT libraries)
foreach(s ${SUBDIRS})
  file(GLOB SUBDIRFILES  ${PROJECT_SOURCE_DIR}/${s}/*.h )
  install(FILES ${SUBDIRFILES}
    DESTINATION include/Klampt/${s}
    COMPONENT headers)
endforeach(s)

# Needs to be last statement:
INCLUDE(CPackSettings)

# Finished:
MESSAGE ("\n")
MESSAGE (STATUS "Compile everything using: make\n")
MESSAGE (STATUS "Compile Klampt lib using: make Klampt\n")
MESSAGE (STATUS "Compile apps using: make apps OR make [appname]")
MESSAGE (STATUS "  Possible values for [appname]:")
MESSAGE (STATUS "    RobotTest, SimTest, RobotPose, MotorCalibrate,")
MESSAGE (STATUS "    URDFtoRob, SimUtil, TrajOpt, Merge, or Pack\n")
MESSAGE (STATUS "Compile examples using: make examples OR make [examplename]")
MESSAGE (STATUS "  Possible values for [examplename]:")
MESSAGE (STATUS "    IKDemo, PlanDemo, DynamicPlanDemo, ContactPlan,")
MESSAGE (STATUS "    UserTrials, or RealTimePlanning\n")
MESSAGE (STATUS "Compile Python API using: make python")
MESSAGE (STATUS "Compile API-documentation using: make docs\n")
MESSAGE (STATUS "Compile Python API-documentation using: make python-docs\n")
MESSAGE (STATUS "Install using: make install")
MESSAGE (STATUS "Uninstall using: make uninstall\n")
