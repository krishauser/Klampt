CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)
include(ExternalProject)

set(PROJECTNAME tests)
project(${PROJECTNAME})

# compile GTest
find_package(GTest)
if (NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main -pthread)
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

#weird workaround to force cmake to build the test executable before running Klampt_Simulation_ODERigidObject
ADD_TEST(ctest_build_test_code "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_ODERigidObject)
SET_TESTS_PROPERTIES ( Klampt_Simulation_ODERigidObject PROPERTIES DEPENDS ctest_build_test_code)

find_package(PythonInterp)

if(PYTHONINTERP_FOUND)
add_test(NAME run_python_tests
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
         COMMAND python -m unittest discover ${CMAKE_SOURCE_DIR})
ENDIF(PYTHONINTERP_FOUND)
