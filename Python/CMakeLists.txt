CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)

if(NOT KLAMPT_ROOT)
  find_package(Klampt)
endif( )

find_program(PYTHON "python")

SET(OUTPUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
SET(DEPS Klampt)

IF(PYTHON)
  configure_file(setup.py.in setup.py)
  add_custom_command(
    OUTPUT ${OUTPUT}/python_timestamp
    COMMAND ${PYTHON} setup.py build_ext
    COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT}/python_timestamp
    DEPENDS ${DEPS}
  )
  add_custom_command(
    OUTPUT ${OUTPUT}/python_docs_timestamp
    COMMAND doxygen klampt/doxygen.conf
    COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT}/python_docs_timestamp
    DEPENDS ${DEPS}
  )
  add_custom_target(python ALL DEPENDS ${OUTPUT}/python_timestamp)
  add_custom_target(python_docs DEPENDS ${OUTPUT}/python_docs_timestamp)
  install(CODE "execute_process(COMMAND ${PYTHON} ${SETUP_PY} install)")
ENDIF(PYTHON)
