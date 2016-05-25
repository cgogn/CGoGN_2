project(cgogn_topology
	LANGUAGES CXX
)

set(HEADER_FILES
	dll.h
        algos/dijkstra.h
)

add_custom_target(cgogn_topology SOURCES ${HEADER_FILES})

install(FILES "dll.h" DESTINATION "include/cgogn/topology")
install(DIRECTORY algos
        DESTINATION include/cgogn/topology
	FILES_MATCHING PATTERN "*.h"
)