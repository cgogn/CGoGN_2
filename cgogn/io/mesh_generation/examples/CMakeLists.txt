cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_io_mesh_gen_examples
	LANGUAGES CXX
)

set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_IMAGES_PATH "${CMAKE_SOURCE_DIR}/data/images/")
add_definitions("-DCGOGN_TEST_IMAGES_PATH=${CGOGN_TEST_IMAGES_PATH}")

set(HEADER_FILES
	program_options.h
	map3_from_image.h
)

set(SOURCE_FILES
	map3_from_image.cpp
	program_options.cpp
)


if (CGOGN_WITH_CGAL_EXAMPLES)
	find_package(Boost QUIET REQUIRED COMPONENTS program_options)


	add_executable(map3_from_image ${HEADER_FILES} ${SOURCE_FILES})
	target_link_libraries(map3_from_image cgogn_core cgogn_io_mesh_generation )


endif()

add_executable(map3_from_surface map3_from_surface.cpp)
target_link_libraries(map3_from_surface cgogn_core cgogn_io cgogn_io_mesh_generation)
