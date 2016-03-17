cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

find_package(CGAL)

project(cgogn_io_mesh_generation
	LANGUAGES CXX
)

set(HEADER_FILES
	c3t3_io.h
)

set(SOURCE_FILES
	c3t3_io.cpp
)

if (CGAL_FOUND)
	add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

	target_include_directories(${PROJECT_NAME} PUBLIC
		$<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
		$<INSTALL_INTERFACE:include/cgogn/io/mesh_generation>
	)
	target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_io)
endif()

if (CGOGN_WITH_CGAL_EXAMPLES)
	add_subdirectory(examples)
endif()
