cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_io_mesh_gen_examples
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_io REQUIRED)

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
	find_package(Boost REQUIRED COMPONENTS program_options)
	add_executable(map3_from_image ${HEADER_FILES} ${SOURCE_FILES})
	target_link_libraries(map3_from_image ${cgogn_core_LIBRARIES} cgogn_io_mesh_generation ${Boost_LIBRARIES})
endif()

add_executable(map3_from_surface map3_from_surface.cpp)
target_link_libraries(map3_from_surface ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} cgogn_io_mesh_generation)
