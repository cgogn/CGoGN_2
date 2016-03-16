cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_io_mesh_gen_examples
	LANGUAGES CXX
)

set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_IMAGES_PATH "${CMAKE_SOURCE_DIR}/data/images/")
add_definitions("-DCGOGN_TEST_IMAGES_PATH=${CGOGN_TEST_IMAGES_PATH}")

if(NOT ${CGOGN_USE_GLIBCXX_DEBUG})
	find_package(Boost QUIET REQUIRED COMPONENTS program_options)
	add_executable(map3_from_image map3_from_image.cpp program_options.h program_options.cpp)
	target_link_libraries(map3_from_image cgogn_core cgogn_io_mesh_generation ${Boost_LIBRARIES} boost_filesystem)
endif()
