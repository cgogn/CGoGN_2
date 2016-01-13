project(${CGOGN_TEST_PREFIX}map
	LANGUAGES CXX
)

add_executable(test_map test_map.cpp)
target_link_libraries(test_map cgogn_core)

#get_property(cgogn_core_includes
#			  TARGET cgogn_core
#			  PROPERTY INCLUDE_DIRECTORIES
#			)
#target_include_directories(test_map PRIVATE ${cgogn_core_includes})

add_executable(test_import test_import.cpp)
target_link_libraries(test_import cgogn_core)

add_executable(test_cmap0 test_cmap0.cpp)
target_link_libraries(test_cmap0 cgogn_core)
