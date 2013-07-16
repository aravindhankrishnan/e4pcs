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


add_executable (test_e4pcs test_e4pcs.cpp config.cpp e4pcs.cpp keypoints_interface.cpp keypoints_interface.h io.cpp
    e4pcs.h typedefs.h io.h config.h)
add_executable (ndt_test ndt_test.cpp io.cpp typedefs.h io.h)
add_executable (icp_test icp_test.cpp io.cpp  typedefs.h io.h)
add_executable (test_keypoint_interface test_keypoint_interface.cpp config.cpp keypoints_interface.cpp
    keypoints_interface.h io.cpp typedefs.h io.h config.h)
add_executable (region_growing_test region_growing_test.cpp io.cpp typedefs.h io.h)
add_executable (icpnl_test icpnl_test.cpp io.cpp typedefs.h io.h)
add_executable (test_sampling test_sampling.cpp io.cpp typedefs.h io.h)
add_executable (downsample downsample.cpp io.cpp io.h)
add_executable (visualize_output visualize_output.cpp io.cpp typedefs.h io.h)
add_executable (generate_transformation_matrix generate_transformation_matrix.cpp)
add_executable (transform_cloud transform_cloud.cpp io.cpp typedefs.h io.h)

target_link_libraries (test_e4pcs ${PCL_LIBRARIES})
target_link_libraries (ndt_test ${PCL_LIBRARIES})
target_link_libraries (icp_test ${PCL_LIBRARIES})
target_link_libraries (test_keypoint_interface ${PCL_LIBRARIES})
target_link_libraries (region_growing_test ${PCL_LIBRARIES})
target_link_libraries (icpnl_test ${PCL_LIBRARIES})
target_link_libraries (test_sampling ${PCL_LIBRARIES})
target_link_libraries (downsample ${PCL_LIBRARIES})
target_link_libraries (visualize_output ${PCL_LIBRARIES})
target_link_libraries (generate_transformation_matrix ${PCL_LIBRARIES})
target_link_libraries (transform_cloud ${PCL_LIBRARIES})
