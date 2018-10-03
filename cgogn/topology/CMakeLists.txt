project(cgogn_topology
	LANGUAGES CXX
)

add_library(${PROJECT_NAME} SHARED "")

add_library(cgogn::topology ALIAS cgogn_topology)

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/dll.h"
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

target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_TOPOLOGY_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${PROJECT_NAME} cgogn::core cgogn::geometry)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/topology
	FILES_MATCHING PATTERN "*.h"
	REGEX "(examples|tests)" EXCLUDE
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}")

# VisualStudio nice source organisation
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
