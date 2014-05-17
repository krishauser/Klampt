#Finds Klamp't package.
#If KLAMPT_ROOT is set, searches for a Klampt install there
#defines 
# - KLAMPT_LIBRARIES
# - KLAMPT_INCLUDE_DIRS
# - KLAMPT_DEFINITIONS
#
# This will properly configure a build to include all external libraries:
# KrisLibrary, its dependencies, ODE, and Klamp't #


INCLUDE(KlamptDependencies)

FIND_LIBRARY( KLAMPT_LIBRARY Klampt
	      PATHS /usr/local/lib "${KLAMPT_ROOT}/lib" )

FIND_PATH(KLAMPT_INCLUDE_DIR
	Simulation/ODESimulator.h
PATHS /usr/include/ /usr/include/Klampt /usr/local/include/Klampt ${KLAMPT_ROOT}
DOC "Directory where Klamp't header files are stored" )


#do the find_package call...
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(KLAMPT "Could not find Klampt " KLAMPT_INCLUDE_DIR KLAMPT_LIBRARY)
# show the BERKELEY_DB_INCLUDE_DIR and BERKELEY_DB_LIBRARIES variables only in the advanced view

#add to dependencies
SET(KLAMPT_LIBRARIES ${KLAMPT_LIBRARY} ${KLAMPT_LIBRARIES})
SET(KLAMPT_INCLUDE_DIRS ${KLAMPT_INCLUDE_DIRS} ${KLAMPT_INCLUDE_DIR})
MARK_AS_ADVANCED(KLAMPT_INCLUDE_DIR KLAMPT_LIBRARY )
MARK_AS_ADVANCED(KLAMPT_INCLUDE_DIRS KLAMPT_LIBRARIES )


