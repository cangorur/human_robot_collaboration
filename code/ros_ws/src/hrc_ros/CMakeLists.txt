cmake_minimum_required(VERSION 2.8.3)
project(hrc_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  message_generation
  tf
)

#### ================= simple-socket --Qichao Xu

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(BOOST_COMPONENTS system coroutine context thread)
# Late 2017 TODO: remove the following checks and always use std::regex
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
        set(BOOST_COMPONENTS ${BOOST_COMPONENTS} regex)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_BOOST_REGEX")
    endif()
endif()
find_package(Boost 1.54.0 COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

if(APPLE)
  set(OPENSSL_ROOT_DIR "/usr/local/opt/openssl")
endif()

#TODO: add requirement for version 1.0.1g (can it be done in one line?)
find_package(OpenSSL REQUIRED)
include_directories(${OPENSSL_INCLUDE_DIR})

find_package(Threads REQUIRED)

include_directories(.)

#### ================= simple-socket --Qichao Xu


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TaskState.msg
  TraySensor.msg
  HumanUpdateMsg.msg
  ObsUpdateMsgIE.msg
  RobotUpdateMsg.msg
  SuccessStatusObserved.msg
  HeadGestureMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   InitiateScenario.srv
   InformHumanAction.srv
   InformHumanState.srv
   InformHumanToTaskMang.srv
   InformObsToTaskMangIE.srv
   InformRobotToTaskMang.srv
   ResetHumanROS.srv
   ResetObsROS.srv
   ResetRobotROS.srv
   MoveNewPackage.srv
   SetConveyorSpeed.srv
   DrawNewStateMC.srv
   SelectPolicy.srv
   PolicySelectorBPR.srv
   InformTrayUpdate.srv
   InformActionRecognized.srv
      
)

# Add the dobot specific services found in the srv/dobot_api_services subfolder  
add_service_files(DIRECTORY srv/dobot_api_services 
FILES 
ContPickAndPlace.srv
InitDobotArmApp.srv
SetPTPCmd.srv
SetQueuedCmdStopExec.srv
SetQueuedCmdForceStopExec.srv
OneTimePickAndPlace.srv
SetPTPCoordinateParams.srv  # service to set speed and velocities of dobot 
SetQueuedCmdStartExec.srv
SetPTPCmd.srv # for the gotoPointApp service 
SetEndEffectorSuctionCup.srv
SetQueuedCmdClear.srv
SimplePickAndPlace.srv

)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES ros_tutorial
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(task_manager_IE
	#src/task_manager/TaskManager.cpp
	#src/task_manager/TaskManagerNode.cpp
        src/task_manager/TaskManager_IExperiment.cpp
	src/task_manager/TaskManagerNode_IExperiment.cpp
	src/helper_functions/Json_parser.cpp)
add_dependencies(task_manager_IE ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(task_manager_IE ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})


add_executable(human_agent
  src/human_agent/HumanAgent.cpp
  src/human_agent/HumanAgentNode.cpp)
add_dependencies(human_agent ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(human_agent ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})


add_executable(robot_agent
  src/robot_agent/RobotAgent.cpp
  src/robot_agent/RobotAgentNode.cpp)
add_dependencies(robot_agent ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robot_agent ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})


add_executable(robot_agent_pomdp_evaluation
  src/robot_agent/RobotAgent_pomdp_eval.cpp
  src/robot_agent/RobotAgentNode.cpp)
add_dependencies(robot_agent_pomdp_evaluation ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robot_agent_pomdp_evaluation ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})


add_executable(robot_motion_agent
  src/robot_motion_agent_IE/RobotMotionAgent_IE.cpp
  src/robot_motion_agent_IE/RobotMotionAgentNode_IE.cpp)
add_dependencies(robot_motion_agent ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robot_motion_agent ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})


add_executable(observation_agent_IE
  #src/observation_agent/ObservationAgent.cpp
  #src/observation_agent/ObservationAgentNode.cpp
  src/observation_agent/ObservationAgentNode_IExperiment.cpp
  src/observation_agent/ObservationAgent_IExperiment.cpp
  src/helper_functions/Json_parser.cpp)
add_dependencies(observation_agent_IE ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(observation_agent_IE ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})


add_executable(test_decision_trigger
  src/test_agents_IE/test_decision_trigger.cpp)
add_dependencies(test_decision_trigger ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(test_decision_trigger ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

#add_executable(test_1_synchronisation_task_observation
#  src/test_agents_IE/test_1_synchronisation_task_observation.cpp)
#add_dependencies(test_1_synchronisation_task_observation ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(test_1_synchronisation_task_observation ${catkin_LIBRARIES} ${Boost_LIBRARIES} #${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_1
  src/test_agents_IE/test_agent.cpp
  src/test_agents_IE/test_1_node_synchro_task_observation.cpp)
add_dependencies(test_1 ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(test_1 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_2
  src/test_agents_IE/test_agent.cpp
  src/test_agents_IE/test_2_node_despot_subtask_behaviour.cpp)
add_dependencies(test_2 ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(test_2 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_3
  src/test_agents_IE/test_agent.cpp
  src/test_agents_IE/test_3_node_despot_subtask_behaviour_constant_subtasks.cpp)
add_dependencies(test_3 ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(test_3 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
