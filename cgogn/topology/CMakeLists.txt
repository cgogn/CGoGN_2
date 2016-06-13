project(cgogn_topology
	LANGUAGES CXX
	)

set(HEADER_FILES
	dll.h
	types/adjacency_cache.h
	types/critical_point.h
	algos/distance_field.h
	algos/features.h
	algos/scalar_field.h
	)

add_custom_target(cgogn_topology SOURCES ${HEADER_FILES})

install(FILES "dll.h" DESTINATION "include/cgogn/topology")
install(DIRECTORY algos
	DESTINATION include/cgogn/topology
	FILES_MATCHING PATTERN "*.h"
	)
