# -*- mode: cmake -*-
#
# this files defines
# - GLPK_INCLUDE_DIR
# - GLPK_LIBRARY
# - GLPK_FOUND

INCLUDE(CheckIncludeFileCXX)
CHECK_INCLUDE_FILE_CXX(glpk.h GLPK_FOUND)


FIND_LIBRARY( GLPK_LIB glpk PATHS /usr/lib $ENV{GLPK_DIR}/lib)
SET(GLPK_LIBRARY ${GLPK_LIB} )

FIND_PATH(GLPK_INCLUDE_DIR
glpk.h
PATHS /usr/include/ /usr/include/glpk $ENV{GLPK_DIR}/include
DOC "Directory where GLPK header files are stored" )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLPK "Could not find GLPK " GLPK_INCLUDE_DIR GLPK_LIBRARY)
# show the BERKELEY_DB_INCLUDE_DIR and BERKELEY_DB_LIBRARIES variables only in the advanced view
MARK_AS_ADVANCED(GLPK_INCLUDE_DIR GLPK_LIBRARY )

