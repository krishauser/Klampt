# Make the MotorCalibrate Qt program
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
PROJECT(MotorCalibrate)

#Building from within Main/[AppName] folder
IF(NOT KLAMPT_ROOT)
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

QT4_WRAP_UI(MotorCalibrate_UISrcs dialog.ui showtext.ui)
QT4_WRAP_CPP(MotorCalibrate_MOCSrcs dialog.h showtext.h)

##### Build target ####
ADD_EXECUTABLE(MotorCalibrate main.cpp dialog.cpp ../motorcalibrate.cpp showtext.cpp ${MotorCalibrate_MOCSrcs} ${MotorCalibrate_UISrcs})
TARGET_LINK_LIBRARIES(MotorCalibrate ${KLAMPT_LIBRARIES} ${QT_LIBRARIES})

#used to ignore main() definition in motorcalibrate.cpp
SET_TARGET_PROPERTIES(MotorCalibrate PROPERTIES
                COMPILE_DEFINITIONS "${QT_DEFINITIONS};HAVE_QT") 

install(TARGETS MotorCalibrate
  DESTINATION bin
  COMPONENT apps)