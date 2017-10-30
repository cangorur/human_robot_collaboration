cmake_minimum_required(VERSION 2.8.3)
project(despot)

set(BINARY_INSTALL_PATH "bin" CACHE PATH "Binary install path")
set(LIBRARY_INSTALL_PATH "lib" CACHE PATH "Library install path")
set(INCLUDE_INSTALL_PATH "include" CACHE PATH "Include install path")
set(CONFIG_INSTALL_PATH "${LIBRARY_INSTALL_PATH}/${PROJECT_NAME}/cmake")

set(DESPOT_BUILD_EXAMPLES ON CACHE BOOL "Build C++ model examples")
set(DESPOT_BUILD_POMDPX ON CACHE BOOL "Build POMDPX example")

set(CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH} "${PROJECT_SOURCE_DIR}/cmake")

#=========
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -msse2 -mfpmath=sse")

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
#=========

include_directories(include)

add_library("${PROJECT_NAME}" SHARED
  src/core/belief.cpp
  src/core/globals.cpp
  src/core/lower_bound.cpp
  src/core/mdp.cpp
  src/core/node.cpp
  src/core/policy.cpp
  src/core/pomdp.cpp
  src/core/solver.cpp
  src/core/upper_bound.cpp
  src/evaluator.cpp
  src/ippc/client.cpp
  src/pomdpx/parser/function.cpp
  src/pomdpx/parser/parser.cpp
  src/pomdpx/parser/variable.cpp
  src/pomdpx/pomdpx.cpp
  src/random_streams.cpp
  src/simple_tui.cpp
  src/solver/aems.cpp
  src/solver/despot.cpp
  src/solver/pomcp.cpp
  src/util/coord.cpp
  src/util/dirichlet.cpp
  src/util/exec_tracker.cpp
  src/util/floor.cpp
  src/util/gamma.cpp
  src/util/logging.cpp
  src/util/random.cpp
  src/util/seeds.cpp
  src/util/util.cpp
  src/util/tinyxml/tinystr.cpp
  src/util/tinyxml/tinyxml.cpp
  src/util/tinyxml/tinyxmlerror.cpp
  src/util/tinyxml/tinyxmlparser.cpp
)
target_link_libraries("${PROJECT_NAME}"
  ${TinyXML_LIBRARIES}
)

#=========
target_link_libraries("${PROJECT_NAME}"
  ${Boost_LIBRARIES}
)
target_link_libraries("${PROJECT_NAME}"
  ${OPENSSL_CRYPTO_LIBRARY}
)
target_link_libraries("${PROJECT_NAME}"
  ${CMAKE_THREAD_LIBS_INIT}
)
#=========


# Build example files
set(DESPOT_BUILD_EXAMPLES FALSE)
if(DESPOT_BUILD_EXAMPLES)
  add_subdirectory(examples/cpp_models/adventurer)
  add_subdirectory(examples/cpp_models/bridge)
  add_subdirectory(examples/cpp_models/chain)
  add_subdirectory(examples/cpp_models/navigation)
  add_subdirectory(examples/cpp_models/pocman)
  add_subdirectory(examples/cpp_models/reg_demo)
  add_subdirectory(examples/cpp_models/rock_sample)
  add_subdirectory(examples/cpp_models/simple_rock_sample)
  add_subdirectory(examples/cpp_models/tag)
  add_subdirectory(examples/cpp_models/tiger)
endif()

if(DESPOT_BUILD_POMDPX)
  add_subdirectory(examples/pomdpx_models)
endif()

install(TARGETS "${PROJECT_NAME}"
  EXPORT "DespotTargets"
  ARCHIVE DESTINATION "${LIBRARY_INSTALL_PATH}"
  LIBRARY DESTINATION "${LIBRARY_INSTALL_PATH}"
  RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
)
install(DIRECTORY "include/${PROJECT_NAME}/"
  DESTINATION "${INCLUDE_INSTALL_PATH}/${PROJECT_NAME}"
)

# Install a DespotConfig.cmake file so CMake can find_package(Despot).
include(CMakePackageConfigHelpers)
configure_package_config_file("cmake/DespotConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/DespotConfig.cmake"
  INSTALL_DESTINATION "${CONFIG_INSTALL_PATH}"
  PATH_VARS INCLUDE_INSTALL_PATH
)

install(EXPORT "DespotTargets"
  FILE "DespotTargets.cmake"
  DESTINATION "${CONFIG_INSTALL_PATH}"
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/DespotConfig.cmake"
  DESTINATION "${CONFIG_INSTALL_PATH}"
)

