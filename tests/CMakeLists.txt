cmake_minimum_required(VERSION 2.8.11)
cmake_policy(SET CMP0040 OLD)
include(ExternalProject)

set(PROJECTNAME tests)
project(${PROJECTNAME})

# compite GTest
find_package(GTest)
if (NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

# add local include
enable_testing()

# gTests for Velocity constraints
SET(TestLibs ${KLAMPT_LIBRARIES} ${GTEST_BOTH_LIBRARIES})


ADD_EXECUTABLE(test_ODERigidObject test_ODERigidObject.cpp)
TARGET_LINK_LIBRARIES(test_ODERigidObject ${TestLibs})
add_dependencies(test_ODERigidObject GTest-ext Klampt python)

add_test(NAME Klampt_Simulation_ODERigidObject
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
         COMMAND test_ODERigidObject)
find_package(PythonInterp)

if(PYTHONINTERP_FOUND)
add_test(NAME run_python_tests
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
         COMMAND python -m unittest discover ${CMAKE_CURRENT_SOURCE_DIR})
ENDIF(PYTHONINTERP_FOUND)
