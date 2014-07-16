# Make the Klamp't examples 
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)
PROJECT(KlamptExamples)

IF(NOT KLAMPT_ROOT)
  SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../)
  SET(KLAMPT_ROOT ../)
  SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
  # COMPILER SETTINGS (default: Release) and flags
  INCLUDE(CompilerSettings)
  FIND_PACKAGE(Klampt REQUIRED)
  ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
  INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})
ENDIF( )


#IKDemo 
SET(EXAMPLES  CartPole ContactPlan PlanDemo DynamicPlanDemo RealTimePlanning SafeSerialClient UserTrials)
ADD_EXECUTABLE(CartPole cartpole.cpp)
ADD_EXECUTABLE(ContactPlan contactplan.cpp)
#ADD_EXECUTABLE(IKDemo ikdemo.cpp)
ADD_EXECUTABLE(PlanDemo plandemo.cpp)
ADD_EXECUTABLE(DynamicPlanDemo dynamicplandemo.cpp)
ADD_EXECUTABLE(RealTimePlanning realtimeplanning.cpp)
ADD_EXECUTABLE(SafeSerialClient safeserialclient.cpp)
ADD_EXECUTABLE(UserTrials usertrials.cpp)
FOREACH(f ${EXAMPLES})
	  TARGET_LINK_LIBRARIES(${f} ${KLAMPT_LIBRARIES})
	  ADD_DEPENDENCIES(${f} Klampt)
ENDFOREACH( )

ADD_CUSTOM_TARGET(examples ALL
		DEPENDS ${EXAMPLES})

#examples install targets
install(TARGETS ${EXAMPLES}
    DESTINATION Examples
    COMPONENT examples)
file(GLOB EXAMPLEFILES *.cpp )
install(FILES ${EXAMPLEFILES}
    DESTINATION Examples
    COMPONENT examples)
install(DIRECTORY ContactPlan
    DESTINATION Examples
    COMPONENT examples)
install(DIRECTORY MotorCalibrateBaxter
    DESTINATION Examples
    COMPONENT examples)
install(DIRECTORY PlanDemo
    DESTINATION Examples
    COMPONENT examples)
