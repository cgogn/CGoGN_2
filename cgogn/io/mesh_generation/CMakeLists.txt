cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

find_package(CGAL)
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
	find_package(MPFR REQUIRED)
	find_package(GMP REQUIRED)
	find_package(Boost QUIET REQUIRED COMPONENTS thread)

	set(	HEADER_FILES
		${HEADER_FILES}
		c3t3_io.h
	)

	set(	SOURCE_FILES
		${SOURCE_FILES}
		c3t3_io.cpp
	)

	set(LIBRARIES "${LIBRARIES};${Boost_LIBRARIES};${GMP_LIBRARIES};${MPFR_LIBRARIES};CGAL;CGAL_ImageIO")
endif()


add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TETGEN_INCLUDE_DIR}>
)

target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${tet_LIBRARIES} ${LIBRARIES})
