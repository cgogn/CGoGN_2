project(cgogn_topology
	LANGUAGES CXX
	)

find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)

set(HEADER_FILES
	dll.h
	types/adjacency_cache.h
	types/critical_point.h
	algos/distance_field.h
	algos/features.h
	algos/scalar_field.h
	)

set(SOURCE_FILES
	algos/distance_field.cpp
	algos/features.cpp
	algos/scalar_field.cpp
	types/adjacency_cache.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_TOPLOGY_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_geometry_LIBRARIES})

install(FILES "dll.h" DESTINATION "include/cgogn/topology")
install(DIRECTORY algos types
	DESTINATION include/cgogn/topology
	FILES_MATCHING PATTERN "*.h"
	)

cgogn_create_package("${CGOGN_SOURCE_DIR}" "include")
