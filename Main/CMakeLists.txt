# Make the Klamp't apps
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
PROJECT(KlamptApps)

IF(NOT KLAMPT_ROOT)
  SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)
  SET(KLAMPT_ROOT ../)
  SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
  # COMPILER SETTINGS (default: Release) and flags
  INCLUDE(CompilerSettings)
  FIND_PACKAGE(Klampt REQUIRED)
  ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
  INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})
ENDIF( )

IF(USE_QT4_APPS)
	MESSAGE(STATUS "Using Qt versions of main apps")
	ADD_SUBDIRECTORY( RobotTestQt )
	ADD_SUBDIRECTORY( SimTestQt )
	ADD_SUBDIRECTORY( RobotPoseQt )
	ADD_SUBDIRECTORY( MotorCalibrateQt )
	ADD_SUBDIRECTORY( URDFtoRobQt )
	IF(WIN32) 
		#need to install Qt dlls
		install(FILES ${QT_BINARY_DIR}/QtCore4.dll
			      ${QT_BINARY_DIR}/QtGui4.dll
			      ${QT_BINARY_DIR}/QtOpenGL4.dll
			      ${QT_BINARY_DIR}/QtSvg4.dll
			      ${QT_BINARY_DIR}/QtXml4.dll
			DESTINATION bin
			COMPONENT apps)
		install(FILES ${QT_PLUGINS_DIR}/iconengines/qsvgicon4.dll
			      DESTINATION bin/plugins/iconengines
			      COMPONENT apps)
	ENDIF(WIN32)
ELSE(USE_QT4_APPS)
	#GLUI GUIs
	MESSAGE(STATUS "Using GLUI/console versions of main apps")
	ADD_EXECUTABLE(RobotTest test.cpp)
	ADD_EXECUTABLE(SimTest simtest.cpp)
	ADD_EXECUTABLE(RobotPose pose.cpp)
	ADD_EXECUTABLE(MotorCalibrate motorcalibrate.cpp)
	ADD_EXECUTABLE(URDFtoRob urdftorob.cpp)
	TARGET_LINK_LIBRARIES(RobotTest ${KLAMPT_LIBRARIES})
	TARGET_LINK_LIBRARIES(SimTest ${KLAMPT_LIBRARIES})
	TARGET_LINK_LIBRARIES(RobotPose ${KLAMPT_LIBRARIES})
	TARGET_LINK_LIBRARIES(MotorCalibrate ${KLAMPT_LIBRARIES})
	TARGET_LINK_LIBRARIES(URDFtoRob ${KLAMPT_LIBRARIES})
	install(TARGETS RobotTest RobotPose SimTest MotorCalibrate URDFtoRob
	  DESTINATION bin
	  COMPONENT apps)
ENDIF(USE_QT4_APPS)
ADD_DEPENDENCIES(RobotTest Klampt)
ADD_DEPENDENCIES(SimTest Klampt)
ADD_DEPENDENCIES(RobotPose Klampt)
ADD_DEPENDENCIES(MotorCalibrate Klampt)
ADD_DEPENDENCIES(URDFtoRob Klampt)

#command line utilities, no need for GUI
ADD_EXECUTABLE(Pack pack.cpp)
ADD_EXECUTABLE(Merge merge.cpp)
ADD_EXECUTABLE(TrajOpt trajopt.cpp)
ADD_EXECUTABLE(SimUtil simutil.cpp)
TARGET_LINK_LIBRARIES(Pack ${KLAMPT_LIBRARIES})
TARGET_LINK_LIBRARIES(Merge ${KLAMPT_LIBRARIES})
TARGET_LINK_LIBRARIES(TrajOpt ${KLAMPT_LIBRARIES})
TARGET_LINK_LIBRARIES(SimUtil ${KLAMPT_LIBRARIES})
ADD_DEPENDENCIES(Pack Klampt)
ADD_DEPENDENCIES(Merge Klampt)
ADD_DEPENDENCIES(TrajOpt Klampt)
ADD_DEPENDENCIES(SimUtil Klampt)

ADD_CUSTOM_TARGET(apps ALL
		DEPENDS RobotTest SimTest RobotPose MotorCalibrate URDFtoRob Pack Merge TrajOpt SimUtil)

