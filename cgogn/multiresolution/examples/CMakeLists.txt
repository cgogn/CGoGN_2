cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_multiresolution_examples
	LANGUAGES CXX
)

set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_subdirectory(cph)
