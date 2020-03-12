project(cgogn_topology
	LANGUAGES CXX
)

# Hide symbols by default
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

add_library(${PROJECT_NAME} "")

add_library(cgogn::topology ALIAS cgogn_topology)

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/types/adjacency_cache.h"
		"${CMAKE_CURRENT_LIST_DIR}/types/critical_point.h"
		"${CMAKE_CURRENT_LIST_DIR}/types/adaptive_tri_quad_cmap2.h"
		"${CMAKE_CURRENT_LIST_DIR}/types/adaptive_tri_quad_cmap2.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/algos/distance_field.h"
		"${CMAKE_CURRENT_LIST_DIR}/algos/features.h"
		"${CMAKE_CURRENT_LIST_DIR}/algos/scalar_field.h"
)

if(${CGOGN_EXTERNAL_TEMPLATES})
	target_sources(${PROJECT_NAME}
		PRIVATE
			"${CMAKE_CURRENT_LIST_DIR}/external_templates.cpp"
	)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
										DEBUG_POSTFIX "_d"
										EXPORT_NAME topology)

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Write out cgogn_topology_export.h to the current binary directory
generate_export_header(cgogn_topology)

target_link_libraries(${PROJECT_NAME} cgogn::core cgogn::geometry)

set(PKG_CONFIG_REQUIRES "cgogn_core cgogn_geometry")
configure_file(${PROJECT_SOURCE_DIR}/cgogn_topology.pc.in ${CMAKE_CURRENT_BINARY_DIR}/cgogn_topology.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cgogn_topology.pc
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
	COMPONENT cgogn_topology_libraries
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/topology
	COMPONENT cgogn_topology_headers
	FILES_MATCHING PATTERN "*.h"
	REGEX "(examples|tests)" EXCLUDE
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cgogn_topology_export.h
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/topology
	COMPONENT cgogn_topology_headers
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}")

# VisualStudio nice source organisation
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
