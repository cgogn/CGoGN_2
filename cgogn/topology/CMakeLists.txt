project(cgogn_topology
	LANGUAGES CXX
)

find_package(OpenNL REQUIRED)

add_library(${PROJECT_NAME} SHARED "")

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
		"${CMAKE_CURRENT_LIST_DIR}/algos/linear_solving.h"
)

if(${CGOGN_EXTERNAL_TEMPLATES})
	target_sources(${PROJECT_NAME}
		PRIVATE
			"${CMAKE_CURRENT_LIST_DIR}/external_templates.cpp"
	)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_TOPOLOGY_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>	
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_OPEN_NL_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
	$<INSTALL_INTERFACE:include/cgogn/thirdparty>
)

target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_geometry ${OpenNL_LIBRARIES})

install(FILES "dll.h" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/topology/)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/algos ${CMAKE_CURRENT_SOURCE_DIR}/types
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/topology
	FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CGOGN_SOURCE_DIR}" "include;include/cgogn/thirdparty")

# VisualStudio nice source organisation
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
