# Make the URDFtoRob Qt program
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4...3.20)
PROJECT(URDFtoRob)

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
  FIND_PACKAGE(Qt5 COMPONENTS Widgets Gui)
  include_directories(${Qt5Widgets_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS})
  add_definitions(${Qt5Widgets_DEFINITIONS} ${Qt5Gui_DEFINITIONS})  
  set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
  set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5Gui_LIBRARIES})

  QT5_WRAP_UI(URDFtoRob_UISrcs dialog.ui)
  QT5_WRAP_CPP(URDFtoRob_MOCSrcs dialog.h)

ELSE(USE_QT5_APPS)
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE(${QT_USE_FILE})

  # Define the app's UI and MOC items
  QT4_WRAP_UI(URDFtoRob_UISrcs dialog.ui)
  QT4_WRAP_CPP(URDFtoRob_MOCSrcs dialog.h)
ENDIF(USE_QT5_APPS)

##### Build target ####
ADD_EXECUTABLE(URDFtoRob main.cpp dialog.cpp ../urdftorob.cpp ${URDFtoRob_MOCSrcs} ${URDFtoRob_UISrcs})
TARGET_LINK_LIBRARIES(URDFtoRob ${KLAMPT_LIBRARIES} ${QT_LIBRARIES})

#used to ignore main() definition in urdftorob.cpp
SET_TARGET_PROPERTIES(URDFtoRob PROPERTIES
                COMPILE_DEFINITIONS "HAVE_QT") 

install(TARGETS URDFtoRob
  DESTINATION bin
  COMPONENT apps)
