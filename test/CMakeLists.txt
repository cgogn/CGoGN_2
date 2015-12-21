cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_test
	LANGUAGES CXX
)

set(CGOGN_TEST_PREFIX "test_")

add_subdirectory(chunk_array)
add_subdirectory(map)

