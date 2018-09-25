cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(bench_tetra_map
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(benchmark REQUIRED)

set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_executable(${PROJECT_NAME} bench_tetra_map.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/thirdparty/google-benchmark/include)
target_link_libraries(${PROJECT_NAME} cgogn::core cgogn::io cgogn::geometry benchmark)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER benchmarks)
