project(${CGOGN_TEST_PREFIX}map
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)

add_executable(map map.cpp)
target_link_libraries(map ${cgogn_core_LIBRARIES})

set_target_properties (map PROPERTIES FOLDER examples/core)
