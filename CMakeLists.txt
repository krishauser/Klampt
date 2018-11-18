CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)
PROJECT( Klampt )

# version (e.g. for packaging)
set(KLAMPT_MAJOR_VERSION 0)
set(KLAMPT_MINOR_VERSION 8)
set(KLAMPT_PATCH_VERSION 0)
set(KLAMPT_VERSION ${KLAMPT_MAJOR_VERSION}.${KLAMPT_MINOR_VERSION}.${KLAMPT_PATCH_VERSION})
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

option(KLAMPT_COMPILE_TESTS "Compile Klampt tests" FALSE)

SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (GLUI_INCLUDE_DIR "/usr/local/include")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
IF(MSVC)
  # needed for some MSVC installations to handle glut32.lib
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SAFESEH:NO")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /SAFESEH:NO")
  set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /SAFESEH:NO")
ENDIF(MSVC)

# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_BINARY_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/Cpp")
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

LINK_DIRECTORIES(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# define the Klamp't library subdirectories in the SUBDIRS variable
SET(SUBDIRS Contact Control IO Interface Modeling Planning Sensing Simulation View)

# Get all compilation files in subdirectories
foreach(s ${SUBDIRS})
  file(GLOB SUBDIR_SRCS  ${PROJECT_SOURCE_DIR}/Cpp/${s}/*.cpp )
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
SET(KLAMPT_INCLUDE_DIRS ${KLAMPT_INCLUDE_DIRS} ${KLAMPT_ROOT}/include)
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})

#show in CMake cache
SET(KLAMPT_INCLUDE_DIRS ${KLAMPT_INCLUDE_DIRS} CACHE STRING "Klamp't include directories" FORCE)
SET(KLAMPT_DEFINITIONS ${KLAMPT_DEFINITIONS} CACHE STRING "Klamp't compiler definitions" FORCE)
SET(KLAMPT_LIBRARIES ${KLAMPT_LIBRARIES} CACHE STRING "Klamp't link libraries" FORCE)

#add Klamp't onto KLAMPT_LIBRARIES, add Klampt/include onto KLAMPT_INCLUDE_DIRS
SET(KLAMPT_LIBRARIES Klampt ${KLAMPT_LIBRARIES})

#needed for python build
IF(WIN32)
  SET(KLAMPT_ABSOLUTE_LIBRARIES ${CMAKE_BINARY_DIR}/lib/libKlampt.a ${KLAMPT_LIBRARIES})
ELSE(WIN32)
  SET(KLAMPT_ABSOLUTE_LIBRARIES ${CMAKE_BINARY_DIR}/lib/libKlampt.lib ${KLAMPT_LIBRARIES})
ENDIF(WIN32)

####### Copy headers from Cpp/* to include/Klampt/* #######
MESSAGE("Copying C++ headers to include/Klampt/*")
foreach(s ${SUBDIRS})
  file(GLOB KLAMPT_SUBDIR_HEADERS
    "${PROJECT_SOURCE_DIR}/Cpp/${s}/*.h"
  )
  file(COPY ${KLAMPT_SUBDIR_HEADERS} DESTINATION include/Klampt/${s}/)
endforeach(s)

####### Klamp't Applications #########
IF(NOT USE_QT5_APPS)
  #USE_QT5_APPS not set
  FIND_PACKAGE(Qt5 COMPONENTS Widgets OpenGL)
  IF(Qt5_FOUND)
    MESSAGE (STATUS "Qt5 detected, building Qt version of apps\n")
    SET(USE_QT5_APPS TRUE)
  ELSE(Qt5_FOUND)
    MESSAGE (STATUS "Qt5 not detected\n")
    SET(USE_QT5_APPS FALSE)
  ENDIF(Qt5_FOUND)
ENDIF(NOT USE_QT5_APPS)

IF(NOT USE_QT5_APPS AND NOT USE_QT4_APPS)
  #USE_QT4_APPS not set
  FIND_PACKAGE(Qt4)
  IF(QT4_FOUND)
    MESSAGE (STATUS "Qt4 detected, building Qt version of apps\n")
    SET(USE_QT4_APPS TRUE)
  ELSE(QT4_FOUND)
    SET(USE_QT4_APPS FALSE)
    MESSAGE (STATUS "Qt not found, building GLUI version of apps\n")
  ENDIF(QT4_FOUND)
ENDIF(NOT USE_QT5_APPS AND NOT USE_QT4_APPS)

ADD_SUBDIRECTORY(Cpp/Main)

####### Klamp't Examples #########
ADD_SUBDIRECTORY(Cpp/Examples)

####### Klamp't Documentation #########
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Documentation/doxygen.conf.in ${CMAKE_CURRENT_BINARY_DIR}/Documentation/doxygen.conf @ONLY)
  ADD_CUSTOM_TARGET(docs ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Documentation/doxygen.conf
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating documentation (Doxygen) at location ${PROJECT_SOURCE_DIR}/doc/html/")
ENDIF(DOXYGEN_FOUND)

# Python API directory
ADD_SUBDIRECTORY(Python)
ADD_DEPENDENCIES(python Klampt)



# uninstall target
configure_file(
    "CMakeModules/CMakeUninstall.cmake.in"
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
  file(GLOB SUBDIRFILES  ${PROJECT_SOURCE_DIR}/Cpp/${s}/*.h )
  install(FILES ${SUBDIRFILES}
    DESTINATION include/Klampt/${s}
    COMPONENT headers)
endforeach(s)
install(DIRECTORY data
    DESTINATION .
    COMPONENT datafiles)
install(DIRECTORY Documentation
    DESTINATION .
    COMPONENT documentation)

#examples and apps already added

IF(WIN32)
	#need to install Assimp, GLPK, glut32 dlls
	install(FILES Library/Assimp32.dll Library/glpk_4_61.dll Library/glut32.dll Library/glew32.dll
		DESTINATION bin
		COMPONENT apps)
ELSE(WIN32)
	#install to /opt/klampt-X.Y
    SET(CMAKE_INSTALL_PREFIX "/opt/klampt-${KLAMPT_MAJOR_VERSION}.${KLAMPT_MINOR_VERSION}.${KLAMPT_PATCH_VERSION}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)

ENDIF(WIN32)

#python demos
install(DIRECTORY Python/demos
    DESTINATION Python
    COMPONENT pythonexamples)
install(DIRECTORY Python/exercises
    DESTINATION Python
    COMPONENT pythonexamples)
install(DIRECTORY Python/control
    DESTINATION Python
    COMPONENT pythonexamples)
install(DIRECTORY Python/utils
    DESTINATION Python
    COMPONENT pythonexamples)

if((NOT WIN32) AND KLAMPT_COMPILE_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

IF(PYTHON)
  install(CODE "execute_process(COMMAND \"cd Python; ${PYTHON} setup.py install\")" COMPONENT python)
ENDIF(PYTHON)


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
