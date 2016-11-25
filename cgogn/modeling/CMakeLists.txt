project(cgogn_modeling
	LANGUAGES CXX
)

set(HEADER_FILES
	dll.h
	algos/catmull_clark.h
	algos/pliant_remeshing.h
	algos/loop.h
	algos/refinements.h
	algos/tetrahedralization.h

	tiling/tiling.h
	tiling/triangular_grid.h
	tiling/triangular_cylinder.h
	tiling/triangular_tore.h
#	tiling/triangular_cube.h

	tiling/square_grid.h
	tiling/square_cylinder.h
	tiling/square_tore.h
#	tiling/square_cube.h
)

set(SOURCE_FILES
	algos/catmull_clark.cpp
	algos/loop.cpp
	algos/pliant_remeshing.cpp
	algos/refinements.cpp
	algos/tetrahedralization.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})


set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_MODELING_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_geometry)

install(FILES "dll.h" DESTINATION "include/cgogn/modeling")
install(DIRECTORY algos tiling
	DESTINATION "include/cgogn/modeling"
	FILES_MATCHING PATTERN "*.h"
)


cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CGOGN_SOURCE_DIR}" "include")
