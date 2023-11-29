CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4...3.20)
PROJECT( KlamptPython )

if(NOT KLAMPT_ROOT)
  SET (KLAMPT_ROOT ${PROJECT_SOURCE_DIR}/.. )
endif( )

# If this was run from /Klampt/ rather than /Klampt/Python, then KLAMPT_LIBRARY will be set
if(NOT KLAMPT_LIBRARY)
  # version (e.g. for packaging)
  set(KLAMPT_MAJOR_VERSION 0)
  set(KLAMPT_MINOR_VERSION 9)
  set(KLAMPT_PATCH_VERSION 1)
  set(KLAMPT_VERSION ${KLAMPT_MAJOR_VERSION}.${KLAMPT_MINOR_VERSION}.${KLAMPT_PATCH_VERSION})

  SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
  find_package(Klampt)
  if(NOT KLAMPT_LIBRARY)
    message(FATAL_ERROR "Klampt library not found in ${KLAMPT_ROOT}")
  endif()
  MESSAGE(STATUS "Klampt library found at ${KLAMPT_LIBRARY}")
  MESSAGE(STATUS "Klampt link libraries: ${KLAMPT_LIBRARIES}")
endif()


#KLAMPT_ABSOLUTE_LIBRARIES needed in setup.py.in
get_filename_component(KLAMPT_ABSOLUTE_LIBRARIES ${KLAMPT_LIBRARY} ABSOLUTE)
SET(KLAMPT_ABSOLUTE_LIBRARIES ${KLAMPT_ABSOLUTE_LIBRARIES} ${KLAMPT_LIBRARIES})

find_program(PYTHON "python")
find_program(PYTHON2 "python2")
find_program(PYTHON3 "python3")

SET(TEMPOUTPUT CMakeFiles)
#make these dependent on all klampt module source files, Klampt, and Klampt's dependency libraries
FILE(GLOB DEPS klampt/src/*.h klampt/src/*.cpp klampt/*.py)
SET(DEPS ${DEPS} ${KLAMPT_LIBRARIES} Klampt)

IF(PYTHON3)

  MESSAGE(STATUS "Configuring Python/setup.py from Python/setup.py.in")
  configure_file(setup.py.in setup.py)
  SET(OUTPUT CMakeFiles)
  MESSAGE(STATUS "Setting up Python/klampt/version.py to ${KLAMPT_VERSION}")
  FILE(WRITE klampt/version.py "__version__ = '${KLAMPT_VERSION}'")

  add_custom_command(
    OUTPUT ${OUTPUT}/python_timestamp
    COMMAND rm -rf build/
    COMMAND ${PYTHON3} -m pip install .
    COMMAND ${CMAKE_COMMAND} -E touch ${TEMPOUTPUT}/python_timestamp
    DEPENDS ${DEPS}
  )
  add_custom_command(
    OUTPUT ${OUTPUT}/python_docs_timestamp
    COMMAND doxygen klampt/doxygen.conf
    COMMAND ${CMAKE_COMMAND} -E touch ${TEMPOUTPUT}/python_docs_timestamp
    DEPENDS ${DEPS}
  )
  add_custom_target(python ALL DEPENDS ${OUTPUT}/python_timestamp)
  add_custom_target(python-docs DEPENDS ${OUTPUT}/python_docs_timestamp)

ENDIF(PYTHON3)
