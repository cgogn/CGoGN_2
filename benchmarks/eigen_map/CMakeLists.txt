cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(bench_eigen_map
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(cgogn_modeling REQUIRED)

add_executable(${PROJECT_NAME} bench_eigen_map.cpp )
target_link_libraries(${PROJECT_NAME} cgogn::core cgogn::modeling)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER benchmarks)
