cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

set (CMAKE_BUILD_TYPE DEBUG)
set (CMAKE_BINARY_DIR  $(CMAKE_SOURCE_DIR)/../build)
set (CMAKE_CXX_COMPILER /usr/bin/clang++)

message ("Source directory : " ${CMAKE_SOURCE_DIR})
message ("Binary directory : " ${CMAKE_BINARY_DIR})

find_package(PCL 1.7 REQUIRED)


find_package(Boost)
if (boost_found)
	include_directories(${BOOST_INCLUDE_DIR})
	add_definitions("-dhas_boost")
endif()

set (UTILS_FOLDER /Volumes/WORKSPACE/projects)

include_directories (/usr/local/include/opencv ${UTILS_FOLDER})
include_directories(${PCL_INCLUDE_DIRS} /usr/local/include )

link_directories (/usr/local/lib/)
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


MESSAGE("PCL includes - ${PCL_INCLUDE_DIRS}")
MESSAGE("PCL Definitions - ${PCL_DEFINITIONS}")


add_executable (test_e4pcs test_e4pcs.cpp e4pcs.cpp io.cpp e4pcs.h typedefs.h io.h)
add_executable (visualize_output visualize_output.cpp io.cpp typedefs.h io.h)

target_link_libraries (test_e4pcs ${PCL_LIBRARIES})
target_link_libraries (visualize_output ${PCL_LIBRARIES})
