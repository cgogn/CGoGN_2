cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_io_examples
	LANGUAGES CXX
)
find_package(cgogn_core REQUIRED)
find_package(cgogn_io REQUIRED)


set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_executable(cmap2_import cmap2_import.cpp)
target_link_libraries(cmap2_import ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES})

add_executable(cmap3_import cmap3_import.cpp)
target_link_libraries(cmap3_import ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES})

add_executable(convert_mesh convert_mesh.cpp)
target_link_libraries(convert_mesh ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES})


set_target_properties(cmap2_import cmap3_import convert_mesh PROPERTIES FOLDER examples/io)