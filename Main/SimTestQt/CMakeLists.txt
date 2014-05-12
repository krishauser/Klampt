# Make the SimTest Qt program
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
PROJECT(SimTest)

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

QT4_WRAP_UI(UISrcs logoptions.ui mainwindow.ui driveredit.ui controllersettings.ui controllercommanddialog.ui connectserial.ui)
QT4_WRAP_CPP(MOCSrcs connectserial.h controllercommanddialog.h driveredit.h controllersettings.h logoptions.h mainwindow.h qsimtestbackend.h qsimtestgui.h ../KlamptQt/qtguibase.h)
QT4_ADD_RESOURCES(RESOURCES icons.qrc)

##### Build target ####
ADD_EXECUTABLE(SimTest main.cpp connectserial.cpp controllercommanddialog.cpp controllersettings.cpp driveredit.cpp logoptions.cpp mainwindow.cpp qsimtestbackend.cpp qsimtestgui.cpp ../KlamptQt/qtguibase.cpp ${MOCSrcs} ${UISrcs} ${RESOURCES})
TARGET_LINK_LIBRARIES(SimTest ${KLAMPT_LIBRARIES} ${QT_LIBRARIES})

