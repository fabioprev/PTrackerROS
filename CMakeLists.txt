cmake_minimum_required(VERSION 2.8.3)

project(PTrackerROS)

set(ROS_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/src/cmake/modules")

find_package(catkin REQUIRED COMPONENTS roscpp LaserScanDetector)
find_package(LibXml2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PTracking REQUIRED)

catkin_package(CATKIN_DEPENDS LaserScanDetector
					  DEPENDS LibXml2
					  DEPENDS OpenCV
					  DEPENDS PTracking
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PTracking_INCLUDE_DIR})

file(GLOB_RECURSE PTrackerROS_src "src/*.cpp")

add_executable(PTrackerROS ${PTrackerROS_src} ${PTracking_INCLUDE_DIR}/PTracker/PTracker.cpp)
target_link_libraries(PTrackerROS ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES} ${OpenCV_LIBS} ${PTracking_LIBRARY})
