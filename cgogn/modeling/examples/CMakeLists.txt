cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_modeling_examples
	LANGUAGES CXX
)

set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_executable(remeshing remeshing.cpp)
target_link_libraries(remeshing cgogn_core cgogn_io cgogn_geometry)
