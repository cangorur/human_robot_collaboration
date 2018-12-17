cmake_minimum_required(VERSION 2.8.3)
project(object_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)


#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

include_directories(.)
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   NineD.msg
   Subfeatures.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Feature.srv
   TrayUpdateCamera.srv
#   InformTrayUpdate.srv
#   InformActionRecognized.srv
#   OneTimePickAndPlace.srv
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
   sensor_msgs
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_tracking
  CATKIN_DEPENDS message_runtime cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
 # ${ZBAR_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/object_tracking.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/object_tracking_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_object_tracking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#set(BOOST_COMPONENTS system coroutine context thread)
# Late 2017 TODO: remove the following checks and always use std::regex
#if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
#    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
#        set(BOOST_COMPONENTS ${BOOST_COMPONENTS} regex)
#        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_BOOST_REGEX")
#    endif()
#endif()
#find_package(Boost 1.54.0 COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
#include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_package( OpenCV REQUIRED )


add_executable(tray_observer src/tray_observer.cpp)
target_link_libraries(tray_observer ${catkin_LIBRARIES})
add_dependencies(tray_observer ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp)

add_executable(nined_receiver src/nined_receiver.cpp)
target_link_libraries(nined_receiver ${catkin_LIBRARIES})
add_dependencies(nined_receiver ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp)

add_executable(classify src/classify.cpp)
target_link_libraries(classify ${catkin_LIBRARIES})
add_dependencies(classify ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp)

add_executable(feature_processer src/feature_processer.cpp)
target_link_libraries(feature_processer ${catkin_LIBRARIES})
add_dependencies(feature_processer ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp)

add_executable(image_show src/image_show.cpp)
add_dependencies(image_show  ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(image_show ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(depth_show src/depth_show.cpp)
add_dependencies(depth_show  ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(depth_show ${catkin_LIBRARIES} ${OpenCV_LIBS})

#include_directories(${ZBAR_INCLUDE_DIR})
add_executable(obj_track src/object_tracking.cpp)
add_dependencies(obj_track  ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(obj_track ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(obj_track ${catkin_LIBRARIES} ${OpenCV_LIBS} zbar)

#target_link_libraries(obj_track ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
