cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_geometry_examples
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(cgogn_rendering REQUIRED)
find_package(QOGLViewer REQUIRED)

set(CMAKE_AUTOMOC ON)

set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")


add_executable(filtering filtering.cpp)
target_link_libraries(filtering ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} ${QOGLViewer_LIBRARIES})
