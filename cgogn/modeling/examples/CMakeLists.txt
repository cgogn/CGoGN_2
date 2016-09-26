cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_modeling_examples
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(cgogn_modeling REQUIRED)

set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_executable(remeshing remeshing.cpp)
target_link_libraries(remeshing ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_geometry_LIBRARIES} ${cgogn_modeling_LIBRARIES})

add_executable(triangular_tiling triangular_tiling.cpp)
target_link_libraries(triangular_tiling ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_geometry_LIBRARIES} ${cgogn_modeling_LIBRARIES})

add_executable(square_tiling square_tiling.cpp)
target_link_libraries(square_tiling ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_geometry_LIBRARIES} ${cgogn_modeling_LIBRARIES})
