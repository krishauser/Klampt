# - Find TinyXML
# Find the native TinyXML includes and library
#
# TINYXML_FOUND - True if TinyXML found.
# TINYXML_INCLUDE_DIR - where to find tinyxml.h, etc.
# TINYXML_LIBRARY - List of libraries when using TinyXML.
#

IF( TINYXML_INCLUDE_DIR )
# Already in cache, be silent
SET( TinyXML_FIND_QUIETLY TRUE )
ENDIF( TINYXML_INCLUDE_DIR )

FIND_PATH( TINYXML_INCLUDE_DIR "tinyxml.h"
  PATHS ${TINYXML_ROOT} )

FIND_LIBRARY( TINYXML_LIBRARY
  NAMES "tinyxml"
  PATHS ${TINYXML_ROOT} )

# handle the QUIETLY and REQUIRED arguments and set TINYXML_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE( "FindPackageHandleStandardArgs" )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( "TinyXML" DEFAULT_MSG TINYXML_INCLUDE_DIR TINYXML_LIBRARY )

MARK_AS_ADVANCED( TINYXML_INCLUDE_DIR TINYXML_LIBRARY )

