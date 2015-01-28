# - Try to find ASSIMP
#
# May wish to define ASSIMP_ROOT
#
# Once done this will define
#  
#  ASSIMP_FOUND        - system has ASSIMP
#  ASSIMP_INCLUDE_DIR  - the ASSIMP include directory
#  ASSIMP_LIBRARY      - Link these to use ASSIMP
#   

IF (ASSIMP_INCLUDE_DIR)
  # Already in cache, be silent
  SET(ASSIMP_FIND_QUIETLY TRUE)
ENDIF (ASSIMP_INCLUDE_DIR)

 # Find the headers
#this is for version 2.x of assimp
# FIND_PATH( ASSIMP_INCLUDE_DIR assimp/assimp.h
#            PATHS /usr/include "${ASSIMP_ROOT}/include" )

#this is for version 3.x of assimp
FIND_PATH( ASSIMP_INCLUDE_DIR assimp/scene.h
            PATHS /usr/include "${ASSIMP_ROOT}/include" )

if( WIN32 )
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(ASSIMP_BUILD_DIR assimp_release-dll_x64)
  ELSE ()
    SET(ASSIMP_BUILD_DIR assimp_release-dll_win32)
  ENDIF ()

  FIND_LIBRARY( ASSIMP_LIBRARY
               NAMES libassimp.lib assimp.lib
               PATHS "C:/libs/assimp/lib"  "${ASSIMP_ROOT}/lib" "${ASSIMP_ROOT}/lib/${ASSIMP_BUILD_DIR}")  

else (WIN32)

 FIND_LIBRARY( ASSIMP_LIBRARY
               NAMES assimp
               PATHS /usr/lib /usr/local/lib "${ASSIMP_ROOT}/lib"  )
endif( WIN32)


IF (ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARY)
  SET(ASSIMP_FOUND TRUE)
ELSE (ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARY)
  SET( ASSIMP_FOUND FALSE )
ENDIF (ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARY)

