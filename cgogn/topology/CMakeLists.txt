project(cgogn_topology
	LANGUAGES CXX
)

find_package(OpenNL REQUIRED)

set(HEADER_TYPES
	types/adjacency_cache.h
	types/critical_point.h
	types/adaptive_tri_quad_cmap2.h
)

set(HEADER_ALGOS
	algos/distance_field.h
	algos/features.h
	algos/scalar_field.h
	algos/linear_solving.h
)

set(HEADER_FILES dll.h ${HEADER_TYPES} ${HEADER_ALGOS})

set(SOURCE_TYPES
	types/adjacency_cache.cpp
	types/adaptive_tri_quad_cmap2.cpp
)

set(SOURCE_ALGOS
	algos/distance_field.cpp
	algos/features.cpp
	algos/scalar_field.cpp
)

set(SOURCE_FILES ${SOURCE_ALGOS} ${SOURCE_TYPES})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_TOPLOGY_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>	
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_OPEN_NL_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
	$<INSTALL_INTERFACE:include/cgogn/thirdparty>
)

target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_geometry ${OpenNL_LIBRARIES})

install(FILES "dll.h" DESTINATION "include/cgogn/topology")
install(DIRECTORY algos types
	DESTINATION "include/cgogn/topology"
	FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CGOGN_SOURCE_DIR}" "include;include/cgogn/thirdparty")

# VisualStudio nice source organisation
source_group("ALGOS" FILES ${HEADER_ALGOS} ${SOURCE_ALGOS}) 
source_group("TYPES" FILES ${HEADER_TYPES} ${SOURCE_TYPES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
