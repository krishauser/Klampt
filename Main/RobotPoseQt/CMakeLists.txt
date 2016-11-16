# Make the RobotPose Qt program
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)
PROJECT(RobotPose)

#Building from within Main/[AppName] folder
IF(NOT KLAMPT_ROOT)
  MESSAGE("No KLAMPT_ROOT set, using ../../")
  SET(KLAMPT_ROOT ../../)
  SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
  INCLUDE(CompilerSettings)
  FIND_PACKAGE(Klampt REQUIRED)
  ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
  INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})
ENDIF( )

INCLUDE_DIRECTORIES(".")
INCLUDE_DIRECTORIES("..")
INCLUDE_DIRECTORIES("../KlamptQt")
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

##### Qt Specific stuff ####
# Configure Qt
SET(QT_USE_QTOPENGL TRUE)
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

# Define the app's UI and MOC items

SET(FRAMES mainwindow collisionoutput playresourceframe resourceframe configsframe floatarrayframe holdframe pathframe stanceframe trimeshframe)
SET(UI_FRAMES)
SET(H_FRAMES)
SET(CPP_FRAMES)
FOREACH(i ${FRAMES})
	  SET(UI_FRAMES ${UI_FRAMES} ${i}.ui)
	  SET(H_FRAMES ${H_FRAMES} ${i}.h)
	  SET(CPP_FRAMES ${CPP_FRAMES} ${i}.cpp)
ENDFOREACH(i ${FRAMES})

QT4_WRAP_UI(RobotPose_UISrcs ${UI_FRAMES} )
QT4_WRAP_CPP(RobotPose_MOCSrcs ${H_FRAMES} ../KlamptQt/qtguibase.h ../KlamptQt/qklamptdisplay.h qresourcetreewidget.h qrobotposegui.h)
QT4_ADD_RESOURCES(RobotPose_Resources ../KlamptQt/icons.qrc)

##### Build target ####
ADD_EXECUTABLE(RobotPose main.cpp ${CPP_FRAMES} qresourcetreewidget.cpp qrobotposegui.cpp ../KlamptQt/qtguibase.cpp ../KlamptQt/qklamptdisplay.cpp resourcemanager.cpp ${RobotPose_MOCSrcs} ${RobotPose_UISrcs} ${RobotPose_Resources})
TARGET_LINK_LIBRARIES(RobotPose ${KLAMPT_LIBRARIES} ${QT_LIBRARIES})

install(TARGETS RobotPose
  DESTINATION bin
  COMPONENT apps)
