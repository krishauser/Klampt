# Make the Klamp't apps
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)
PROJECT(KlamptApps)

IF(NOT KLAMPT_ROOT)
  SET(KLAMPT_ROOT ../../)
  SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${KLAMPT_ROOT})
  SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
  # COMPILER SETTINGS (default: Release) and flags
  INCLUDE(CompilerSettings)
  FIND_PACKAGE(Klampt REQUIRED)
  ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
  INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})
ENDIF( )

EXEC_PROGRAM(python2.7-config ARGS --includes OUTPUT_VARIABLE PY_CXX)
EXEC_PROGRAM(python2.7-config ARGS --libs     OUTPUT_VARIABLE PY_LD)
MESSAGE (STATUS "Python CXX flags: " ${PY_CXX})
MESSAGE (STATUS "Python LD flags: " ${PY_LD})

set(CMAKE_CXX_FLAGS        "-fpermissive")
SET(CMAKE_CXX_FLAGS        "${CMAKE_CXX_FLAGS} ${PY_CXX}")

#Hack to force QT 5.5 on my system. TODO - make more robust
SET(CMAKE_PREFIX_PATH /home/motion/Qt/5.6/gcc_64)
find_package(Qt5WebSockets REQUIRED)
set(CMAKE_AUTOMOC ON)

ADD_EXECUTABLE(WebServer main.cpp WebsocketServer.cpp PythonWrapper.cpp)

TARGET_LINK_LIBRARIES(WebServer ${KLAMPT_LIBRARIES})
TARGET_LINK_LIBRARIES(WebServer Qt5::WebSockets)
TARGET_LINK_LIBRARIES(WebServer ${PY_LD})

ADD_DEPENDENCIES(WebServer Klampt)

install(TARGETS WebServer
	DESTINATION bin
	COMPONENT apps)





