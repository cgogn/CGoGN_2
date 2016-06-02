cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

find_package(cgogn_core REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(tet REQUIRED)

project(cgogn_io_mesh_generation
	LANGUAGES CXX
)

set(LIBRARIES "")

set(HEADER_FILES
	tetgen_structure_io.h
)

set(SOURCE_FILES
	tetgen_structure_io.cpp
)

if (CGOGN_WITH_CGAL_EXAMPLES)
	find_package(CGAL REQUIRED COMPONENTS Core ImageIO)
	find_package(MPFR REQUIRED)
	find_package(GMP REQUIRED)
	find_package(Boost REQUIRED COMPONENTS thread)

	set(	HEADER_FILES
		${HEADER_FILES}
		c3t3_io.h
	)

	set(	SOURCE_FILES
		${SOURCE_FILES}
		c3t3_io.cpp
		)

	link_directories(${Boost_LIBRARY_DIRS})
endif (CGOGN_WITH_CGAL_EXAMPLES)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TETGEN_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGAL_INCLUDE_DIRS}>
	$<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
	$<BUILD_INTERFACE:${GMP_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${MPFR_INCLUDE_DIR}>
)

target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${tet_LIBRARIES})
if (CGOGN_WITH_CGAL_EXAMPLES)
	target_link_libraries(${PROJECT_NAME} CGAL::CGAL_ImageIO CGAL::CGAL_Core ${Boost_LIBRARIES} ${MPFR_LIBRARIES} ${GMP_LIBRARIES})
endif()
