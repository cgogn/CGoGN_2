cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_test
	LANGUAGES CXX
)

set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_executable(test_import test_import.cpp)
target_link_libraries(test_import cgogn_core)