cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

set (CMAKE_BUILD_TYPE DEBUG)
set (CMAKE_BINARY_DIR  $(CMAKE_SOURCE_DIR)/../build)
set (CMAKE_CXX_COMPILER /usr/bin/clang++)



find_package(PCL 1.7 REQUIRED)


find_package(Boost)
if (boost_found)
	include_directories(${BOOST_INCLUDE_DIR})
	add_definitions("-dhas_boost")
endif()


include_directories (/Volumes/WORKSPACE/packages/ann_1.1.2/include /usr/local/include/opencv /Volumes/WORKSPACE/projects)
include_directories(${PCL_INCLUDE_DIRS} /usr/local/include /Volumes/WORKSPACE/softwares/boost_1_53_0)

link_directories (/Volumes/WORKSPACE/packages/ann_1.1.2/lib /usr/local/lib/)
link_directories(${PCL_LIBRARY_DIRS} /Volumes/WORKSPACE/softwares/boost_1_53_0/stage/lib)

add_definitions(${PCL_DEFINITIONS})


MESSAGE("PCL includes - ${PCL_INCLUDE_DIRS}")
MESSAGE("PCL Definitions - ${PCL_DEFINITIONS}")


add_executable (test_e4pcs test_e4pcs.cpp e4pcs.cpp io.cpp e4pcs.h typedefs.h io.h)

target_link_libraries (test_e4pcs ${PCL_LIBRARIES})
