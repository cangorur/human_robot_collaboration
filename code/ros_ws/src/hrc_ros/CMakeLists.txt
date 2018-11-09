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
  ObsUpdateMsg.msg
  RobotUpdateMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   InitiateScenario.srv
   InformHumanAction.srv
   InformHumanState.srv
   InformHumanToTaskMang.srv
   InformObsToTaskMang.srv
   InformRobotToTaskMang.srv
   ResetHumanROS.srv
   ResetObsROS.srv
   ResetRobotROS.srv
   MoveNewPackage.srv
   SetConveyorSpeed.srv
   DrawNewStateMC.srv
   SelectPolicy.srv
   PolicySelectorBPR.srv
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

add_executable(task_manager
	src/task_manager/TaskManager.cpp
	src/task_manager/TaskManagerNode.cpp)
add_dependencies(task_manager ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(task_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})


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


add_executable(observation_agent
  src/observation_agent/ObservationAgent.cpp
  src/observation_agent/ObservationAgentNode.cpp)
add_dependencies(observation_agent ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(observation_agent ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
