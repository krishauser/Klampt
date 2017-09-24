# - Find Open Motion Planning Library
# Find the native OMPL includes and library
#
# OMPL_FOUND - True if OMPL found.
# OMPL_INCLUDE_DIR - where to find OMPL.h, etc.
# OMPL_LIBRARIES - List of libraries when using OMPL.
#

IF( OMPL_INCLUDE_DIR )
# Already in cache, be silent
SET( OMPL_FIND_QUIETLY TRUE )
ENDIF( OMPL_INCLUDE_DIR )

FIND_PATH( OMPL_INCLUDE_DIR "ompl/config.h"
  PATHS ${OMPL_ROOT} )

FIND_LIBRARY( OMPL_LIBRARIES
  NAMES "ompl"
  PATHS ${OMPL_ROOT} )

# handle the QUIETLY and REQUIRED arguments and set OMPL_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE( "FindPackageHandleStandardArgs" )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( "OMPL" DEFAULT_MSG OMPL_INCLUDE_DIR OMPL_LIBRARIES )

MARK_AS_ADVANCED( OMPL_INCLUDE_DIR OMPL_LIBRARIES )

