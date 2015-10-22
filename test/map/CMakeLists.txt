project(${CGOGN_TEST_PREFIX}map
		LANGUAGES CXX
	   )

add_executable(test_map test_map.cpp)
target_link_libraries(test_map cgogn_core)

