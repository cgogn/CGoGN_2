cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_tiling_examples
	LANGUAGES CXX
)

set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_executable(triangular_tiling triangular_tiling.cpp)
target_link_libraries(triangular_tiling cgogn_core cgogn_io)

