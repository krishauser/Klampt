# Make the RobotPose Qt program
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4...3.20)
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

# Configure Qt
IF(USE_QT5_APPS)
  FIND_PACKAGE(Qt5 COMPONENTS Widgets Gui OpenGL)
  include_directories(${Qt5Widgets_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
  add_definitions(${Qt5Widgets_DEFINITIONS} ${Qt5Gui_DEFINITIONS} ${Qt5OpenGL_DEFINITIONS})  
  set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
  set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5OpenGL_LIBRARIES})

  QT5_WRAP_UI(RobotPose_UISrcs ${UI_FRAMES} )
  QT5_WRAP_CPP(RobotPose_MOCSrcs  ${H_FRAMES} ../KlamptQt/qtguibase.h ../KlamptQt/qklamptguibase.h ../KlamptQt/qklamptdisplay.h qresourcetreewidget.h qrobotposegui.h)
  QT5_ADD_RESOURCES(RobotPose_Resources ../KlamptQt/icons.qrc)
ELSE(USE_QT5_APPS)
  SET(QT_USE_QTOPENGL TRUE)
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE(${QT_USE_FILE})

  QT4_WRAP_UI(RobotPose_UISrcs ${UI_FRAMES} )
  QT4_WRAP_CPP(RobotPose_MOCSrcs  ${H_FRAMES} ../KlamptQt/qtguibase.h ../KlamptQt/qklamptguibase.h ../KlamptQt/qklamptdisplay.h qresourcetreewidget.h qrobotposegui.h)
  QT4_ADD_RESOURCES(RobotPose_Resources ../KlamptQt/icons.qrc)
ENDIF(USE_QT5_APPS)

##### Build target ####
ADD_EXECUTABLE(RobotPose main.cpp ${CPP_FRAMES} qresourcetreewidget.cpp qrobotposegui.cpp ../KlamptQt/qklamptguibase.cpp ../KlamptQt/qtguibase.cpp ../KlamptQt/qklamptdisplay.cpp resourcemanager.cpp ${RobotPose_MOCSrcs} ${RobotPose_UISrcs} ${RobotPose_Resources})
TARGET_LINK_LIBRARIES(RobotPose ${KLAMPT_LIBRARIES} ${QT_LIBRARIES})

install(TARGETS RobotPose
  DESTINATION bin
  COMPONENT apps)
