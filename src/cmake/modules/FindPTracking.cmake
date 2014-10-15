# - Try to find PTracking include dirs and libraries
#	
#	Usage of this module as follows:
#	
#	  find_package(PTracking)
#	
#	Variables defined by this module:
#	
#	  PTracking_FOUND			System has PTracking, include and library dirs found
#	  PTracking_INCLUDE_DIR		The PTracking include directories.
#	  PTracking_LIBRARY			The PTracking library

find_path(PTracking_ROOT_DIR NAMES PTracking/Core/Filters/ObjectParticleFilter.h)
set(PTracking_ROOT_DIR ${PTracking_ROOT_DIR}/PTracking)
find_path(PTracking_INCLUDE_DIR NAMES Core/Filters/ObjectParticleFilter.h HINTS ${PTracking_ROOT_DIR})

find_library(PTracking_LIBRARY NAMES ptracking HINTS ${PTracking_ROOT_DIR}/../../lib/PTracking)

if (PTracking_LIBRARY STREQUAL "PTracking_LIBRARY-NOTFOUND")
	if (PTracking_FIND_REQUIRED)
		message(FATAL_ERROR "PTracking library has NOT been found. Did you install the library?.")
	endif (PTracking_FIND_REQUIRED)
	
	set(PTracking_FOUND 0)
else (PTracking_LIBRARY STREQUAL "PTracking_LIBRARY-NOTFOUND")
	message(STATUS "Found PTracking")
	set(PTracking_FOUND 1)
endif (PTracking_LIBRARY STREQUAL "PTracking_LIBRARY-NOTFOUND")
