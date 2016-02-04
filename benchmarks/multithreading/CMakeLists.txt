cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(bench_multithreading
	LANGUAGES CXX
)

set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_executable(${PROJECT_NAME} bench_multithreading.cpp)
target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_io cgogn_geometry)
