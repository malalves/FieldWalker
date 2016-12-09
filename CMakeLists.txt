cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS "/usr/include/gazebo-7/gazebo/msgs/proto")

set (msgs
Evolve.proto
EvolveRequest.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(evolve_robots_msgs SHARED ${PROTO_SRCS})
target_link_libraries(evolve_robots_msgs ${PROTOBUF_LIBRARY})


