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
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

##### Qt Specific stuff ####
# Configure Qt
SET(QT_USE_QTOPENGL TRUE)
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

# Define the app's UI and MOC items

QT4_WRAP_UI(RobotPose_UISrcs collisionoutput.ui mainwindow.ui playresourceframe.ui resourceframe.ui)
QT4_WRAP_CPP(RobotPose_MOCSrcs collisionoutput.h mainwindow.h playresourceframe.h qrobotposebackend.h qrobotposegui.h ../KlamptQt/qtguibase.h resourceframe.h)
QT4_ADD_RESOURCES(RobotPose_Resources ../KlamptQt/icons.qrc)

##### Build target ####
ADD_EXECUTABLE(RobotPose main.cpp collisionoutput.cpp mainwindow.cpp playresourceframe.cpp qresourcetreeitem.cpp qrobotposebackend.cpp qrobotposegui.cpp ../KlamptQt/qtguibase.cpp resourceframe.cpp resourcemanager.cpp ${RobotPose_MOCSrcs} ${RobotPose_UISrcs} ${RobotPose_Resources})
TARGET_LINK_LIBRARIES(RobotPose ${KLAMPT_LIBRARIES} ${QT_LIBRARIES})

