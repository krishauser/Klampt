CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

if(NOT KLAMPT_ROOT)
  SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../CMakeModules")
  SET (KLAMPT_ROOT ${PROJECT_SOURCE_DIR}/.. )
  find_package(Klampt)
endif( )

find_program(PYTHON "python")

SET(TEMPOUTPUT CMakeFiles)
SET(DEPS Klampt)

IF(PYTHON)
  configure_file(setup.py.in setup.py)
  SET(OUTPUT CMakeFiles)
  add_custom_command(
    OUTPUT ${OUTPUT}/python_timestamp
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
  install(CODE "execute_process(COMMAND ${PYTHON} setup.py install)")
  add_custom_target(python-install DEPENDS ${OUTPUT}/python_install_timestamp)
ENDIF(PYTHON)
