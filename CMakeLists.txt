CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4...3.20)
PROJECT( Klampt )

# version (e.g. for packaging)
set(KLAMPT_MAJOR_VERSION 0)
set(KLAMPT_MINOR_VERSION 9)
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
SET(KLAMPT_INCLUDE_DIRS ${KLAMPT_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}/include)
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})

#show in CMake cache
SET(KLAMPT_INCLUDE_DIRS ${KLAMPT_INCLUDE_DIRS} CACHE STRING "Klamp't include directories" FORCE)
SET(KLAMPT_DEFINITIONS ${KLAMPT_DEFINITIONS} CACHE STRING "Klamp't compiler definitions" FORCE)
SET(KLAMPT_LIBRARIES ${KLAMPT_LIBRARIES} CACHE STRING "Klamp't link libraries" FORCE)

#needed for python build
if(WIN32)
  SET(KLAMPT_ABSOLUTE_LIBRARIES ${CMAKE_BINARY_DIR}/lib/Release/Klampt.lib ${KLAMPT_LIBRARIES})
else(WIN32)
  SET(KLAMPT_ABSOLUTE_LIBRARIES "${CMAKE_BINARY_DIR}/lib/libKlampt.a" ${KLAMPT_LIBRARIES})
endif(WIN32)

#add Klamp't onto KLAMPT_LIBRARIES, add Klampt/include onto KLAMPT_INCLUDE_DIRS
SET(KLAMPT_LIBRARIES Klampt ${KLAMPT_LIBRARIES})

#helper function
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
   UNSET(listVar)
ENDFUNCTION(PREPEND)

IF(WIN32)
	#need to install Assimp, GLPK, glew dlls
  SET(KLAMPT_EXTRA_DLLS assimp.dll glpk_4_61.dll glew32.dll)
  PREPEND(KLAMPT_EXTRA_DLLS_ABS ${KLAMPT_DEPENDENCY_LIB_DIR} ${KLAMPT_EXTRA_DLLS})
  SET(KLAMPT_EXTRA_DLLS_ABS ${KLAMPT_EXTRA_DLLS_ABS} CACHE STRING "Extra DLLs for windows build")
  install(FILES ${KLAMPT_EXTRA_DLLS_ABS} DESTINATION bin COMPONENT apps)
ELSE(WIN32)
	#install to /opt/klampt-X.Y.Z
    SET(CMAKE_INSTALL_PREFIX "/opt/klampt-${KLAMPT_MAJOR_VERSION}.${KLAMPT_MINOR_VERSION}.${KLAMPT_PATCH_VERSION}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)
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

####### Klamp't Documentation #########
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/Cpp/docs/doxygen.conf.in ${CMAKE_SOURCE_DIR}/Cpp/docs/doxygen.conf @ONLY)
  ADD_CUSTOM_TARGET(docs ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Cpp/docs/doxygen.conf
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating documentation (Doxygen) at location ${CMAKE_SOURCE_DIR}/Cpp/docs/html/")
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
install(DIRECTORY Documentation
    DESTINATION .
    COMPONENT documentation)

#examples and apps already added

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
FOREACH(app ${APPS})
	MESSAGE (STATUS "    " ${app})
ENDFOREACH()
MESSAGE (STATUS "Compile Python API using: make python")
MESSAGE (STATUS "Compile API-documentation using: make docs\n")
MESSAGE (STATUS "Compile Python API-documentation using: make python-docs\n")
MESSAGE (STATUS "Install using: make install")
MESSAGE (STATUS "Uninstall using: make uninstall\n")
