cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(bench_quad_map
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(benchmark REQUIRED)

add_executable(${PROJECT_NAME} bench_quad_map.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/thirdparty/google-benchmark/include)
target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_geometry_LIBRARIES} ${benchmark_LIBRARIES})
