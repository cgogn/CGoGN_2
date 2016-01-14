project(${CGOGN_TEST_PREFIX}map
	LANGUAGES CXX
)

add_executable(map map.cpp)
target_link_libraries(map cgogn_core)

#get_property(cgogn_core_includes
#			  TARGET cgogn_core
#			  PROPERTY INCLUDE_DIRECTORIES
#			)
#target_include_directories(test_map PRIVATE ${cgogn_core_includes})

