# -*- mode: cmake -*-
#
# Looks for environment variable GLPK_DIR or CMake variable GLPK_ROOT
# In windows, may need to set GLPK_MAJOR_VERSION / GLPK_MINOR_VERSION
#
# this files defines
# - GLPK_INCLUDE_DIR
# - GLPK_LIBRARY
# - GLPK_FOUND

INCLUDE(CheckIncludeFileCXX)
CHECK_INCLUDE_FILE_CXX(glpk.h GLPK_FOUND)

IF(WIN32)
  SET(GLPK_MAJOR_VERSION 4)
  SET(GLPK_MINOR_VERSION 52)
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    FIND_LIBRARY( GLPK_LIBRARY "glpk_${GLPK_MAJOR_VERSION}_${GLPK_MINOR_VERSION}" PATHS $ENV{GLPK_DIR}/w64 ${GLPK_ROOT}/w64)
  ELSE(CMAKE_SIZEOF_VOID_P EQUAL 8)
    FIND_LIBRARY( GLPK_LIBRARY "glpk_${GLPK_MAJOR_VERSION}_${GLPK_MINOR_VERSION}" PATHS $ENV{GLPK_DIR}/w32 ${GLPK_ROOT}/w32)
  ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)

  FIND_PATH(GLPK_INCLUDE_DIR
  glpk.h
  PATHS $ENV{GLPK_DIR}/src ${GLPK_ROOT}/src
  DOC "Directory where GLPK header files are stored" )

ELSE (WIN32)
  FIND_LIBRARY( GLPK_LIBRARY glpk PATHS /usr/lib $ENV{GLPK_DIR}/lib ${GLPK_ROOT}/lib)

  FIND_PATH(GLPK_INCLUDE_DIR
  glpk.h
  PATHS /usr/include/ /usr/include/glpk $ENV{GLPK_DIR}/include ${GLPK_ROOT}/include
  DOC "Directory where GLPK header files are stored" )
ENDIF(WIN32)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLPK "Could not find GLPK " GLPK_INCLUDE_DIR GLPK_LIBRARY)
# show the BERKELEY_DB_INCLUDE_DIR and BERKELEY_DB_LIBRARIES variables only in the advanced view
MARK_AS_ADVANCED(GLPK_INCLUDE_DIR GLPK_LIBRARY )

