CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

if(NOT KLAMPT_ROOT)
  SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../CMakeModules")
  SET (KLAMPT_ROOT ${PROJECT_SOURCE_DIR}/.. )
  find_package(Klampt)
endif( )

find_program(PYTHON "python")
find_program(PYTHON3 "python3")

SET(TEMPOUTPUT CMakeFiles)
#make these dependent on all klampt module source files, Klampt, and Klampt's dependency libraries
FILE(GLOB DEPS klampt/src/*.h klampt/src/*.cpp klampt/*.py)
SET(DEPS ${DEPS} ${KLAMPT_LIBRARIES} Klampt)

IF(PYTHON)
  MESSAGE(STATUS "Configuring {BUILD}/Python/setup.py from Python/setup.py.in")
  configure_file(setup.py.in setup.py)
  SET(OUTPUT CMakeFiles)
  add_custom_command(
    OUTPUT ${OUTPUT}/python_timestamp
    COMMAND rm -rf build/
    COMMAND ${PYTHON} setup.py build_ext
    COMMAND ${CMAKE_COMMAND} -E touch ${TEMPOUTPUT}/python_timestamp
    DEPENDS ${DEPS}
  )
  add_custom_command(
    OUTPUT ${OUTPUT}/python_docs_timestamp
    COMMAND doxygen klampt/doxygen.conf
    COMMAND ${CMAKE_COMMAND} -E touch ${TEMPOUTPUT}/python_docs_timestamp
    DEPENDS ${DEPS}
  )
  add_custom_command(
    OUTPUT ${OUTPUT}/python_install_timestamp
    COMMAND ${PYTHON} setup.py install
    COMMAND ${CMAKE_COMMAND} -E touch ${TEMPOUTPUT}/python_install_timestamp
    DEPENDS ${DEPS}
  )
  add_custom_target(python ALL DEPENDS ${OUTPUT}/python_timestamp)
  add_custom_target(python-docs DEPENDS ${OUTPUT}/python_docs_timestamp)
  add_custom_target(python-install DEPENDS ${OUTPUT}/python_install_timestamp python)

  add_custom_command(
    OUTPUT python3_version
    COMMAND mkdir python3_version
    COMMAND cp -r klampt/data python3_version/klampt/
    COMMAND 2to3 --output-dir=python3_version/klampt -W -n klampt
  )
  add_custom_command(
    OUTPUT ${OUTPUT}/python3_timestamp
    COMMAND ${PYTHON3} setup.py build_ext
    COMMAND ${CMAKE_COMMAND} -E touch ${TEMPOUTPUT}/python3_timestamp
    DEPENDS ${DEPS} python3_version
  )
  add_custom_command(
    OUTPUT ${OUTPUT}/python3_install_timestamp
    COMMAND ${PYTHON3} setup.py install
    COMMAND ${CMAKE_COMMAND} -E touch ${TEMPOUTPUT}/python3_install_timestamp
    DEPENDS ${DEPS}
  )
  add_custom_target(python3 ALL DEPENDS ${OUTPUT}/python3_timestamp)
  add_custom_target(python3-install ALL DEPENDS ${OUTPUT}/python3_install_timestamp python3)
ENDIF(PYTHON)
