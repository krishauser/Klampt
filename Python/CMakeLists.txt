CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

if(NOT KLAMPT_ROOT)
  SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../CMakeModules")
  SET (KLAMPT_ROOT ${PROJECT_SOURCE_DIR}/.. )
  find_package(Klampt)
endif( )

find_program(PYTHON "python")
find_program(PYTHON2 "python2")
find_program(PYTHON3 "python3")

SET(TEMPOUTPUT CMakeFiles)
#make these dependent on all klampt module source files, Klampt, and Klampt's dependency libraries
FILE(GLOB DEPS klampt/src/*.h klampt/src/*.cpp klampt/*.py)
SET(DEPS ${DEPS} ${KLAMPT_LIBRARIES} Klampt)

IF(PYTHON)

  MESSAGE(STATUS "Configuring Python/setup.py from Python/setup.py.in")
  configure_file(setup.py.in setup.py)
  MESSAGE(STATUS "Configuring Python/docs/source/conf.py from Python/docs/source/conf.py.in")
  configure_file(docs/source/conf.py.in docs/source/conf.py.in @ONLY)
  SET(OUTPUT CMakeFiles)

  add_custom_command(
    OUTPUT ${OUTPUT}/python_timestamp
    COMMAND rm -rf build/
    COMMAND ${PYTHON3} setup.py build_ext
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
    COMMAND ${PYTHON3} setup.py install
    COMMAND ${CMAKE_COMMAND} -E touch ${TEMPOUTPUT}/python_install_timestamp
    DEPENDS ${DEPS}
  )
  add_custom_target(python ALL DEPENDS ${OUTPUT}/python_timestamp)
  add_custom_target(python-docs DEPENDS ${OUTPUT}/python_docs_timestamp)
  add_custom_target(python-install DEPENDS ${OUTPUT}/python_install_timestamp python)

  add_custom_command(
    OUTPUT python2_version
    COMMAND mkdir python2_version
    COMMAND cp -r klampt/data python2_version/klampt/
    COMMAND 2to3 --output-dir=python2_version/klampt -W -n klampt
  )
  add_custom_command(
    OUTPUT ${OUTPUT}/python2_timestamp
    COMMAND ${PYTHON2} setup.py build_ext
    COMMAND ${CMAKE_COMMAND} -E touch ${TEMPOUTPUT}/python2_timestamp
    DEPENDS ${DEPS} python2_version
  )
  add_custom_command(
    OUTPUT ${OUTPUT}/python2_install_timestamp
    COMMAND ${PYTHON2} setup.py install
    COMMAND ${CMAKE_COMMAND} -E touch ${TEMPOUTPUT}/python2_install_timestamp
    DEPENDS ${DEPS}
  )
  #add_custom_target(python2 ALL DEPENDS ${OUTPUT}/python2_timestamp)
  #add_custom_target(python2-install ALL DEPENDS ${OUTPUT}/python2_install_timestamp python2)
ENDIF(PYTHON)
