project(cgogn_modeling
	LANGUAGES CXX
)

set(HEADER_ALGOS
	algos/catmull_clark.h
	algos/curves.h
        algos/doo_sabin.h
	algos/loop.h
	algos/refinements.h
	algos/pliant_remeshing.h
	algos/decimation.h
	algos/tetrahedralization.h
	algos/dual.h
)
set(HEADER_TILING
	tiling/tiling.h
	tiling/triangular_grid.h
	tiling/triangular_cylinder.h
	tiling/triangular_tore.h
	tiling/square_grid.h
	tiling/square_cylinder.h
	tiling/square_tore.h
)
set(HEADER_DECIMATION
	decimation/edge_approximator.h
	decimation/edge_approximator_mid_edge.h
	decimation/edge_approximator_qem.h
	decimation/edge_traversor_map_order.h
	decimation/edge_traversor_edge_length.h
	decimation/edge_traversor_qem.h
)

set(HEADER_FILES dll.h ${HEADER_ALGOS} ${HEADER_TILING} ${HEADER_DECIMATION})

set(SOURCE_ALGOS
	algos/catmull_clark.cpp
        algos/doo_sabin.cpp
	algos/loop.cpp
	algos/refinements.cpp
	algos/pliant_remeshing.cpp
	algos/decimation.cpp
	algos/tetrahedralization.cpp
)
set(SOURCE_TILING
	tiling/tiling.cpp
	tiling/triangular_grid.cpp
	tiling/triangular_cylinder.cpp
	tiling/triangular_tore.cpp
#	tiling/triangular_cube.cpp
	tiling/square_grid.cpp
	tiling/square_cylinder.cpp
	tiling/square_tore.cpp
#	tiling/square_cube.h
)
set(SOURCE_FILES ${SOURCE_ALGOS} ${SOURCE_TILING})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_MODELING_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_geometry)

install(FILES "dll.h" DESTINATION "include/cgogn/modeling")
install(DIRECTORY algos tiling decimation
	DESTINATION "include/cgogn/modeling"
	FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CGOGN_SOURCE_DIR}" "include")

source_group("ALGOS" FILES ${HEADER_ALGOS} ${SOURCE_ALGOS})
source_group("TILING" FILES ${HEADER_TILING} ${SOURCE_TILING})
source_group("DECIMATION" FILES ${HEADER_DECIMATION})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
