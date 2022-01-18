# Make the Klamp't apps
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4...3.20)
PROJECT(KlamptApps)

IF(NOT KLAMPT_ROOT)
  SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../../bin)
  SET(KLAMPT_ROOT ../../)
  SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
  # COMPILER SETTINGS (default: Release) and flags
  INCLUDE(CompilerSettings)
  FIND_PACKAGE(Klampt REQUIRED)
  ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
  INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})
ENDIF( )

IF(USE_QT5_APPS)
	MESSAGE(STATUS "Using Qt versions of main apps")
	ADD_SUBDIRECTORY( RobotTestQt )
	ADD_SUBDIRECTORY( SimTestQt )
	ADD_SUBDIRECTORY( RobotPoseQt )
	ADD_SUBDIRECTORY( MotorCalibrateQt )
	ADD_SUBDIRECTORY( URDFtoRobQt )
	IF(WIN32) 
		#need to install Qt dlls
		#QT_BINARY_DIR and QT_PLUGINS_DIR are not defined in Qt5... this is a bit of a hack
		get_filename_component(QT_BINARY_DIR "${Qt5Core_DIR}/../../../bin" ABSOLUTE)
		get_filename_component(QT_PLUGINS_DIR "${Qt5Core_DIR}/../../../plugins" ABSOLUTE)

		install(FILES ${QT_BINARY_DIR}/Qt5Core.dll
			      ${QT_BINARY_DIR}/Qt5Gui.dll
			      ${QT_BINARY_DIR}/Qt5Widgets.dll
			      ${QT_BINARY_DIR}/Qt5OpenGL.dll
			      ${QT_BINARY_DIR}/Qt5Svg.dll
			DESTINATION bin
			COMPONENT apps)
		install(FILES ${QT_PLUGINS_DIR}/platforms/qwindows.dll
			      DESTINATION bin/platforms
			      COMPONENT apps)
		install(FILES ${QT_PLUGINS_DIR}/iconengines/qsvgicon.dll
			      DESTINATION bin/iconengines
			      COMPONENT apps)
		install(FILES ${QT_PLUGINS_DIR}/imageformats/qjpeg.dll ${QT_PLUGINS_DIR}/imageformats/qgif.dll ${QT_PLUGINS_DIR}/imageformats/qsvg.dll
			      DESTINATION bin/imageformats
			      COMPONENT apps)
	ENDIF(WIN32)
	SET(APPS RobotTest SimTest RobotPose MotorCalibrate URDFtoRob)
ELSEIF(USE_QT4_APPS)
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
			      DESTINATION bin/iconengines
			      COMPONENT apps)
		install(FILES ${QT_PLUGINS_DIR}/imageformats/qjpeg4.dll ${QT_PLUGINS_DIR}/imageformats/qgif4.dll ${QT_PLUGINS_DIR}/imageformats/qsvg4.dll
			      DESTINATION bin/imageformats
			      COMPONENT apps)
	ENDIF(WIN32)
	SET(APPS RobotTest SimTest RobotPose MotorCalibrate URDFtoRob)
ELSEIF(GLUI_FOUND)
	#GLUI GUIs
	MESSAGE(STATUS "Using GLUI/console versions of main apps")
	ADD_EXECUTABLE(RobotTest test.cpp)
	ADD_EXECUTABLE(SimTest simtest.cpp)
	ADD_EXECUTABLE(RobotPose pose.cpp)
	ADD_EXECUTABLE(MotorCalibrate motorcalibrate.cpp)
	ADD_EXECUTABLE(URDFtoRob urdftorob.cpp)
	SET(MAINAPPS RobotTest SimTest RobotPose MotorCalibrate URDFtoRob)
ELSE(USE_QT5_APPS)
	MESSAGE(STATUS "No visualization framework (Qt/GLUI) available, using console versions of main apps")
	ADD_EXECUTABLE(MotorCalibrate motorcalibrate.cpp)
	ADD_EXECUTABLE(URDFtoRob urdftorob.cpp)
	SET(MAINAPPS MotorCalibrate URDFtoRob)
ENDIF(USE_QT5_APPS)

#command line utilities, no need for GUI
ADD_EXECUTABLE(Pack pack.cpp)
ADD_EXECUTABLE(Merge merge.cpp)
ADD_EXECUTABLE(TrajOpt trajopt.cpp)
ADD_EXECUTABLE(SimUtil simutil.cpp)
SET(MAINAPPS ${MAINAPPS} Pack Merge TrajOpt SimUtil)

foreach(app ${MAINAPPS}) 
  TARGET_LINK_LIBRARIES(${app} ${KLAMPT_LIBRARIES})
  install(TARGETS ${app}
	DESTINATION bin
	COMPONENT apps)
endforeach()

SET(APPS ${APPS} ${MAINAPPS})

foreach(app ${APPS})
  ADD_DEPENDENCIES(${app} Klampt)
endforeach()

ADD_CUSTOM_TARGET(apps ALL
		DEPENDS ${APPS})

