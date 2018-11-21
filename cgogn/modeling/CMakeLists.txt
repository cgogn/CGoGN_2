project(cgogn_modeling
	LANGUAGES CXX
)

# Hide symbols by default
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

add_library(${PROJECT_NAME} SHARED "")

add_library(cgogn::modeling ALIAS cgogn_modeling)

target_sources(${PROJECT_NAME}
	PRIVATE
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
		"${CMAKE_CURRENT_LIST_DIR}/tiling/hexa_volume.h"

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

set_target_properties(${PROJECT_NAME} PROPERTIES
										DEBUG_POSTFIX "_d"
										EXPORT_NAME modeling)

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Write out cgogn_core_export.h to the current binary directory
include(GenerateExportHeader)
generate_export_header(cgogn_modeling)

target_link_libraries(${PROJECT_NAME} cgogn::core cgogn::geometry)

set(PKG_CONFIG_REQUIRES "cgogn_core cgogn_geometry")
configure_file(${PROJECT_SOURCE_DIR}/cgogn_modeling.pc.in ${CMAKE_CURRENT_BINARY_DIR}/cgogn_modeling.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cgogn_modeling.pc
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
	COMPONENT cgogn_modeling_libraries
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/modeling
	COMPONENT cgogn_modeling_headers
	FILES_MATCHING PATTERN "*.h"
	REGEX "(examples|tests)" EXCLUDE
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cgogn_modeling_export.h
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/modeling
	COMPONENT cgogn_modeling_headers
)


cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}")

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
