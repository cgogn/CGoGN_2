cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_io_examples
	LANGUAGES CXX
)

set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_executable(cmap2_import cmap2_import.cpp)
target_link_libraries(cmap2_import cgogn_core)
