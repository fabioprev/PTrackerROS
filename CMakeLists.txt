cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/src/cmake/modules")

find_package(LibXml2 REQUIRED)
find_package(PTracking REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${PTracking_INCLUDE_DIR})

file(GLOB_RECURSE PTrackerROS_src "src/*.cpp")

message("PTracking libs: ${PTracking_LIBRARY}")

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(PTrackerROS ${PTrackerROS_src} ${PTracking_INCLUDE_DIR}/PTracker/PTracker.cpp)
target_link_libraries(PTrackerROS ${LIBXML2_LIBRARIES} ${PTracking_LIBRARY})
