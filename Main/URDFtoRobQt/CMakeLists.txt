# Make the URDFtoRob Qt program
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
PROJECT(URDFtoRob)

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

QT4_WRAP_UI(URDFtoRob_UISrcs dialog.ui)
QT4_WRAP_CPP(URDFtoRob_MOCSrcs dialog.h)

##### Build target ####
ADD_EXECUTABLE(URDFtoRob main.cpp dialog.cpp ../urdftorob.cpp ${URDFtoRob_MOCSrcs} ${URDFtoRob_UISrcs})
TARGET_LINK_LIBRARIES(URDFtoRob ${KLAMPT_LIBRARIES} ${QT_LIBRARIES})

#used to ignore main() definition in urdftorob.cpp
SET_TARGET_PROPERTIES(URDFtoRob PROPERTIES
                COMPILE_DEFINITIONS "${QT_DEFINITIONS};HAVE_QT") 
