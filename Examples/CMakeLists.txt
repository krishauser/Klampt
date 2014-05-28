# Make the Klamp't examples 
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
PROJECT(KlamptExamples)

IF(NOT KLAMPT_ROOT)
  SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)
  SET(KLAMPT_ROOT ../)
  SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
  # COMPILER SETTINGS (default: Release) and flags
  INCLUDE(CompilerSettings)
  FIND_PACKAGE(Klampt REQUIRED)
  ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
  INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})
ENDIF( )


#IKDemo 
SET(EXAMPLES  CartPole ContactPlan PlanDemo DynamicPlanDemo RealTimePlanning UserTrials)
ADD_EXECUTABLE(CartPole cartpole.cpp)
ADD_EXECUTABLE(ContactPlan contactplan.cpp)
#ADD_EXECUTABLE(IKDemo ikdemo.cpp)
ADD_EXECUTABLE(PlanDemo plandemo.cpp)
ADD_EXECUTABLE(DynamicPlanDemo dynamicplandemo.cpp)
ADD_EXECUTABLE(RealTimePlanning realtimeplanning.cpp)
ADD_EXECUTABLE(UserTrials usertrials.cpp)
FOREACH(f ${EXAMPLES})
	  TARGET_LINK_LIBRARIES(${f} ${KLAMPT_LIBRARIES})
	  ADD_DEPENDENCIES(${f} Klampt)
ENDFOREACH( )

ADD_CUSTOM_TARGET(examples ALL
		DEPENDS ${EXAMPLES})
