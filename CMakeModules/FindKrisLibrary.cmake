#Finds KrisLibrary package.
#If KRISLIBRARY_ROOT is set, searches for a KrisLibrary install there
#defines 
# - KRISLIBRARY_LIBRARIES
# - KRISLIBRARY_INCLUDE_DIRS
# - KRISLIBRARY_DEFINITIONS
#
# This will properly configure a build to include all external libraries:
# KrisLibrary and its dependencies

SET(KRISLIBRARY_LIBRARIES "")
SET(KRISLIBRARY_INCLUDE_DIRS "")
SET(KRISLIBRARY_DEFINITIONS "")

#this will get everything but KrisLibrary
INCLUDE(KrisLibraryDependencies)

# Find KrisLibrary
FIND_LIBRARY( KRISLIBRARY_LIBRARY KrisLibrary PATHS /usr/local/lib "${KRISLIBRARY_ROOT}/KrisLibrary/lib" ${KRISLIBRARY_ROOT})
FIND_PATH(KRISLIBRARY_INCLUDE_DIR
 	KrisLibrary/myfile.h
        PATHS /usr/include /usr/local/include ${KRISLIBRARY_ROOT}
         DOC "Directory where KrisLibrary header files are stored" )

#do the find_package call...
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(KRISLIBRARY "Could not find KrisLibrary " KRISLIBRARY_INCLUDE_DIR KRISLIBRARY_LIBRARY)

SET(KRISLIBRARY_INCLUDE_DIRS ${KRISLIBRARY_INCLUDE_DIRS} ${KRISLIBRARY_INCLUDE_DIR})
SET(KRISLIBRARY_LIBRARIES ${KRISLIBRARY_LIBRARY} ${KRISLIBRARY_LIBRARIES})

MARK_AS_ADVANCED(KRISLIBRARY_INCLUDE_DIR KRISLIBRARY_LIBRARY )
MARK_AS_ADVANCED(KRISLIBRARY_INCLUDE_DIRS KRISLIBRARY_LIBRARIES )


