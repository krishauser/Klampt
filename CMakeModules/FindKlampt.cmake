#Finds Klamp't package.
#If KLAMPT_ROOT is set, searches for a Klampt install there
#defines 
# - KLAMPT_LIBRARIES_DEBUG
# - KLAMPT_LIBRARIES_RELEASE
# - KLAMPT_INCLUDE_DIRS
# - KLAMPT_DEFINITIONS
#
# This will properly configure a build to include all external libraries:
# KrisLibrary, its dependencies, ODE, and Klamp't #


INCLUDE(KlamptDependencies)

FIND_PATH(KLAMPT_INCLUDE_DIR
	Klampt/Simulation/ODESimulator.h
PATHS /usr/include/ /usr/local/include ${KLAMPT_ROOT}/include
DOC "Directory where Klamp't header files are stored" )


if(WIN32)
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(KLAMPT_BUILD_DIR msvc64)
  ELSE ()
    SET(KLAMPT_BUILD_DIR msvc)
  ENDIF ()

  FIND_LIBRARY( KLAMPT_LIBRARY_DEBUG Klampt
               PATHS "${KLAMPT_ROOT}/${KLAMPT_BUILD_DIR}/lib/Debug" "${KLAMPT_ROOT}/lib/Debug" "${KLAMPT_ROOT}/lib")
  FIND_LIBRARY( KLAMPT_LIBRARY_RELEASE Klampt
               PATHS "${KLAMPT_ROOT}/${KLAMPT_BUILD_DIR}/lib/Release" "${KLAMPT_ROOT}/lib/Release" "${KLAMPT_ROOT}/lib")  
  #this is used to pick between RELEASE and DEBUG library
  #Note: do not use SelectLibraryConfigurations, this clobbers KRISLIBRARY_LIBRARIES
  SET(KLAMPT_LIBRARY debug ${KLAMPT_LIBRARY_DEBUG} optimized ${KLAMPT_LIBRARY_RELEASE})
  
  find_package_handle_standard_args(KLAMPT
	DEFAULT_MSG
	KLAMPT_INCLUDE_DIR
	KLAMPT_LIBRARY_DEBUG
	KLAMPT_LIBRARY_RELEASE)
else()
  FIND_LIBRARY( KLAMPT_LIBRARY Klampt
	      PATHS /usr/local/lib "${KLAMPT_ROOT}/lib")
endif()

#do the find_package call...
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(KLAMPT "Could not find Klampt " KLAMPT_INCLUDE_DIR KLAMPT_LIBRARY)

#add to dependencies
SET(KLAMPT_LIBRARIES ${KLAMPT_LIBRARY} ${KLAMPT_LIBRARIES})
SET(KLAMPT_INCLUDE_DIRS ${KLAMPT_INCLUDE_DIRS} ${KLAMPT_INCLUDE_DIR})
MARK_AS_ADVANCED(KLAMPT_LIBRARY )
MARK_AS_ADVANCED(KLAMPT_INCLUDE_DIR )
MARK_AS_ADVANCED(KLAMPT_INCLUDE_DIRS KLAMPT_LIBRARY_DEBUG KLAMPT_LIBRARY_RELEASE)

SET(KLAMPT_INCLUDE_DIRS ${KLAMPT_INCLUDE_DIRS} CACHE STRING "Klamp't include directories" FORCE)
SET(KLAMPT_LIBRARIES_DEBUG ${KLAMPT_LIBRARIES_DEBUG} CACHE STRING "Klamp't link libraries for Debug build" FORCE)
SET(KLAMPT_LIBRARIES_RELEASE ${KLAMPT_LIBRARIES_RELEASE} CACHE STRING "Klamp't link libraries for Release build" FORCE)
SET(KLAMPT_DEFINITIONS ${KLAMPT_DEFINITIONS} CACHE STRING "Klamp't compiler definitions" FORCE)

