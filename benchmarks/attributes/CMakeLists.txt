cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_rendering_examples
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(cgogn_modeling REQUIRED)

set(CMAKE_AUTOMOC ON)

set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")


add_executable(bench_attrib bench_attrib.cpp)
target_link_libraries(bench_attrib ${cgogn_core_LIBRARIES} ${cgogn_modeling_LIBRARIES} )
