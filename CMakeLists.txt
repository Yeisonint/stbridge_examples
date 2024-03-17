cmake_minimum_required(VERSION 3.22)
project(st_bridge_examples)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -std=gnu++20")

include(CheckIncludeFileCXX)
check_include_file_cxx("format" CXX_FORMAT_SUPPORT)
if (NOT CXX_FORMAT_SUPPORT)
    find_package(fmt REQUIRED)
endif()

file(GLOB_RECURSE stlink_bridge_sources
    ${CMAKE_SOURCE_DIR}/bridge/STLINK-V3-BRIDGE/src/*/*.cpp 
    ${CMAKE_SOURCE_DIR}/bridge/STLINK-V3-BRIDGE/src/*/*.*.h
)

include_directories(${CMAKE_SOURCE_DIR}/bridge/STLINK-V3-BRIDGE/src/bridge)
include_directories(${CMAKE_SOURCE_DIR}/bridge/STLINK-V3-BRIDGE/src/common)
include_directories(${CMAKE_SOURCE_DIR}/bridge/STLINK-V3-BRIDGE/src/error)
include_directories(${CMAKE_SOURCE_DIR}/include)

ADD_EXECUTABLE(main src/main.cpp ${stlink_bridge_sources})
target_link_libraries(main ${CMAKE_SOURCE_DIR}/drivers/linux_x64/libSTLinkUSBDriver.so)
if (NOT CXX_FORMAT_SUPPORT)
    target_link_libraries(main fmt::fmt)
endif()