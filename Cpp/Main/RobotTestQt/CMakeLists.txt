# Make the RobotTest Qt program
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4...3.20)
PROJECT(RobotTest)

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
IF(USE_QT5_APPS)
  FIND_PACKAGE(Qt5 COMPONENTS Widgets Gui OpenGL)
  include_directories(${Qt5Widgets_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
  add_definitions(${Qt5Widgets_DEFINITIONS} ${Qt5Gui_DEFINITIONS} ${Qt5OpenGL_DEFINITIONS})  
  set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
  set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5OpenGL_LIBRARIES})

  # Define the app's UI and MOC items
  QT5_WRAP_UI(RobotTest_UISrcs collisionoutput.ui mainwindow.ui)
  QT5_WRAP_CPP(RobotTest_MOCSrcs collisionoutput.h mainwindow.h ../KlamptQt/qklamptguibase.h ../KlamptQt/qklamptdisplay.h ../KlamptQt/qtguibase.h qrobottestgui.h)
  QT5_ADD_RESOURCES(RobotTest_Resources ../KlamptQt/icons.qrc)
ELSE(USE_QT5_APPS)
  SET(QT_USE_QTOPENGL TRUE)
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE(${QT_USE_FILE})

  # Define the app's UI and MOC items
  QT4_WRAP_UI(RobotTest_UISrcs collisionoutput.ui mainwindow.ui)
  QT4_WRAP_CPP(RobotTest_MOCSrcs collisionoutput.h mainwindow.h ../KlamptQt/qklamptguibase.h ../KlamptQt/qklamptdisplay.h ../KlamptQt/qtguibase.h qrobottestgui.h)
  QT4_ADD_RESOURCES(RobotTest_Resources ../KlamptQt/icons.qrc)
ENDIF(USE_QT5_APPS)

##### Build target ####
ADD_EXECUTABLE(RobotTest main.cpp collisionoutput.cpp mainwindow.cpp ../KlamptQt/qklamptguibase.cpp ../KlamptQt/qklamptdisplay.cpp ../KlamptQt/qtguibase.cpp qrobottestgui.cpp ${RobotTest_MOCSrcs} ${RobotTest_UISrcs} ${RobotTest_Resources})
TARGET_LINK_LIBRARIES(RobotTest ${KLAMPT_LIBRARIES} ${QT_LIBRARIES})

install(TARGETS RobotTest
  DESTINATION bin
  COMPONENT apps)
