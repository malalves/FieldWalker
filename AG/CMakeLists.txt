# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11 -fpermissive ${GAZEBO_CXX_FLAGS}")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_executable (Gazebo_evolve Gazebo_evolve.cc)
target_link_libraries(Gazebo_evolve evolve_robots_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(Gazebo_evolve evolve_robots_msgs)

add_library(line_follow SHARED line_follow.cc)
target_link_libraries(line_follow ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})