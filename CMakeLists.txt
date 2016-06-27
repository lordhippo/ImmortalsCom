cmake_minimum_required(VERSION 3.0)
project(ImmortalsCom)
include("${CMAKE_SOURCE_DIR}/CMake/Helpers.cmake")

set_project_paths()
enable_parallel_build()

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

file(GLOB_RECURSE GEN_PROTO_DEF "*.proto")

add_protobuf_generate_command(GEN_PROTO_DEF PROTO_SRCS PROTO_HDRS)

file(GLOB_RECURSE GEN_SOURCE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "Source/*.c" "Source/*.cpp" "/Source/*.cc" "/Source/*.cxx")
file(GLOB_RECURSE GEN_HEADER_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "Source/*.h" "Source/*.hpp")

file(GLOB_RECURSE GEN_DEP_SOURCE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "3rdparty/*.c" "3rdparty/*.cpp" "/3rdparty/*.cc" "/3rdparty/*.cxx")
file(GLOB_RECURSE GEN_DEP_HEADER_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "3rdparty/*.h" "3rdparty/*.hpp")

set(SOURCE_FILES
    ${GEN_HEADER_FILES}
    ${PROTO_HDRS}
    ${GEN_DEP_HEADER_FILES}
    ${GEN_SOURCE_FILES}
    ${PROTO_SRCS}
    ${GEN_DEP_SOURCE_FILES})

add_executable(ImmortalsCom ${SOURCE_FILES})

set_files_group(SOURCE_FILES)

if(MSVC)
  target_link_libraries(ImmortalsCom wsock32 ws2_32)
endif()

target_link_libraries(ImmortalsCom ${PROTOBUF_LIBRARIES})