cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(bench_attrib
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(cgogn_modeling REQUIRED)

set(CMAKE_AUTOMOC ON)

add_executable(${PROJECT_NAME} bench_attrib.cpp)
target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_modeling_LIBRARIES} )

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER benchmarks)
