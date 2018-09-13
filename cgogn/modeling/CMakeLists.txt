project(cgogn_modeling
	LANGUAGES CXX
)

add_library(${PROJECT_NAME} SHARED "")

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/dll.h"
		"${CMAKE_CURRENT_LIST_DIR}/algos/catmull_clark.h"
		"${CMAKE_CURRENT_LIST_DIR}/algos/curves.h"
		"${CMAKE_CURRENT_LIST_DIR}/algos/doo_sabin.h"
		"${CMAKE_CURRENT_LIST_DIR}/algos/loop.h"
		"${CMAKE_CURRENT_LIST_DIR}/algos/refinements.h"
		"${CMAKE_CURRENT_LIST_DIR}/algos/pliant_remeshing.h"
		"${CMAKE_CURRENT_LIST_DIR}/algos/decimation.h"
		"${CMAKE_CURRENT_LIST_DIR}/algos/tetrahedralization.h"
		"${CMAKE_CURRENT_LIST_DIR}/algos/tetrahedralization.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/algos/dual.h"

		"${CMAKE_CURRENT_LIST_DIR}/tiling/tiling.h"
		"${CMAKE_CURRENT_LIST_DIR}/tiling/triangular_grid.h"
		"${CMAKE_CURRENT_LIST_DIR}/tiling/triangular_cylinder.h"
		"${CMAKE_CURRENT_LIST_DIR}/tiling/triangular_tore.h"
		"${CMAKE_CURRENT_LIST_DIR}/tiling/square_grid.h"
		"${CMAKE_CURRENT_LIST_DIR}/tiling/square_cylinder.h"
		"${CMAKE_CURRENT_LIST_DIR}/tiling/square_tore.h"

		"${CMAKE_CURRENT_LIST_DIR}/decimation/edge_approximator.h"
		"${CMAKE_CURRENT_LIST_DIR}/decimation/edge_approximator_mid_edge.h"
		"${CMAKE_CURRENT_LIST_DIR}/decimation/edge_approximator_qem.h"
		"${CMAKE_CURRENT_LIST_DIR}/decimation/edge_traversor_map_order.h"
		"${CMAKE_CURRENT_LIST_DIR}/decimation/edge_traversor_edge_length.h"
		"${CMAKE_CURRENT_LIST_DIR}/decimation/edge_traversor_qem.h"
)

if(${CGOGN_EXTERNAL_TEMPLATES})
	target_sources(${PROJECT_NAME}
		PRIVATE
			"${CMAKE_CURRENT_LIST_DIR}/external_templates.cpp"
	)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_MODELING_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_geometry)

install(FILES "dll.h" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/modeling)
install(DIRECTORY algos tiling decimation
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/modeling
	FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CGOGN_SOURCE_DIR}" "include")

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
