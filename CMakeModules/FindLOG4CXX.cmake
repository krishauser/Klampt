# - Try to find LOG4CXX
#
# May wish to define LOG4CXX_ROOT
#
# Once done this will define
#  
#  LOG4CXX_FOUND        - system has LOG4CXX
#  LOG4CXX_INCLUDE_DIR  - the LOG4CXX include directory
#  LOG4CXX_LIBRARY      - Link these to use LOG4CXX
#  LOG4CXX_DEFINITIONS  - Will define the LOG4CXX version
#   

IF (LOG4CXX_INCLUDE_DIR)
  # Already in cache, be silent
  SET(LOG4CXX_FIND_QUIETLY TRUE)
ENDIF( LOG4CXX_INCLUDE_DIR)

 # Find the headers

#this is for version 2.x of LOG4CXX
FIND_PATH( LOG4CXX_INCLUDE_DIR log4cxx/log4cxx.h
            PATHS /usr/include "${LOG4CXX_ROOT}/include" )


if( WIN32 )


  FIND_LIBRARY( LOG4CXX_LIBRARY
               NAMES log4cxx.lib
               PATHS "C:/libs/LOG4CXX/lib"  "${LOG4CXX_ROOT}/lib" "${LOG4CXX_ROOT}/lib/${LOG4CXX_BUILD_DIR}")  

  FIND_LIBRARY( APR_LIBRARY
               NAMES apr-1.lib
               PATHS "C:/libs/LOG4CXX/lib"  "${LOG4CXX_ROOT}/lib" "${LOG4CXX_ROOT}/lib/${LOG4CXX_BUILD_DIR}")  
  FIND_LIBRARY( APRUTIL_LIBRARY
               NAMES aprutil-1.lib
               PATHS "C:/libs/LOG4CXX/lib"  "${LOG4CXX_ROOT}/lib" "${LOG4CXX_ROOT}/lib/${LOG4CXX_BUILD_DIR}")  
  FIND_LIBRARY( LOG4CXX_XML_LIBRARY
               NAMES xml.lib
               PATHS "C:/libs/LOG4CXX/lib"  "${LOG4CXX_ROOT}/lib" "${LOG4CXX_ROOT}/lib/${LOG4CXX_BUILD_DIR}")  

			   
			   
else (WIN32)

  FIND_LIBRARY( LOG4CXX_LIBRARY
               NAMES log4cxx
               PATHS "C:/libs/LOG4CXX/lib"  "${LOG4CXX_ROOT}/lib" "${LOG4CXX_ROOT}/lib/${LOG4CXX_BUILD_DIR}")  
  FIND_LIBRARY( APR_LIBRARY
               NAMES apr-1
               PATHS "C:/libs/LOG4CXX/lib"  "${LOG4CXX_ROOT}/lib" "${LOG4CXX_ROOT}/lib/${LOG4CXX_BUILD_DIR}")  
  FIND_LIBRARY( APRUTIL_LIBRARY
               NAMES aprutil-1
               PATHS "C:/libs/LOG4CXX/lib"  "${LOG4CXX_ROOT}/lib" "${LOG4CXX_ROOT}/lib/${LOG4CXX_BUILD_DIR}")  
  FIND_LIBRARY( LOG4CXX_XML_LIBRARY
               NAMES xml2
               PATHS "C:/libs/LOG4CXX/lib"  "${LOG4CXX_ROOT}/lib" "${LOG4CXX_ROOT}/lib/${LOG4CXX_BUILD_DIR}")  

endif( WIN32)

SET(LOG4CXX_LIBRARIES ${LOG4CXX_LIBRARY} ${APR_LIBRARY} ${APRUTIL_LIBRARY} ${LOG4CXX_XML_LIBRARY})
#other things log4cxx depends on
if(WIN32)
SET(LOG4CXX_LIBRARIES ${LOG4CXX_LIBRARIES} WS2_32.Lib MsWSock.Lib AdvAPI32.Lib odbc32.lib)
endif(WIN32)

IF (LOG4CXX_INCLUDE_DIR AND LOG4CXX_LIBRARY)
  SET(LOG4CXX_FOUND TRUE)
ELSE (LOG4CXX_INCLUDE_DIR AND LOG4CXX_LIBRARY)
  SET( LOG4CXX_FOUND FALSE )
ENDIF (LOG4CXX_INCLUDE_DIR AND LOG4CXX_LIBRARY)

