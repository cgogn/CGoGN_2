cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

find_package(CGAL)
find_package(Boost QUIET COMPONENTS thread)
project(cgogn_io_mesh_generation
	LANGUAGES CXX
)

set(HEADER_FILES
	map3_from_image.h
	c3t3_io.h
)

set(SOURCE_FILES
	c3t3_io.cpp
)

if (${CGAL_FOUND} AND ${Boost_FOUND})
	find_package(MPFR REQUIRED)
	find_package(GMP REQUIRED)
	add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

	target_include_directories(${PROJECT_NAME} PUBLIC
		$<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
		$<INSTALL_INTERFACE:include/cgogn/io/mesh_generation>
	)
	target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_io CGAL CGAL_ImageIO ${Boost_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})

	add_subdirectory(examples)
endif()
