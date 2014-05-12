# Make the RobotPose Qt program
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
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

##### Qt Specific stuff ####
# Configure Qt
SET(QT_USE_QTOPENGL TRUE)
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

# Define the app's UI and MOC items

QT4_WRAP_UI(UISrcs collisionoutput.ui mainwindow.ui)
QT4_WRAP_CPP(MOCSrcs collisionoutput.h mainwindow.h qtguibase.h qrobottestbackend.h qrobottestgui.h)

##### Build target ####
ADD_EXECUTABLE(RobotTest main.cpp collisionoutput.cpp mainwindow.cpp qtguibase.cpp qrobottestbackend.cpp qrobottestgui.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(RobotTest ${KLAMPT_LIBRARIES} ${QT_LIBRARIES})

