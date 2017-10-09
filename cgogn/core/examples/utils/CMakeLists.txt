project(${CGOGN_TEST_PREFIX}utils
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)


add_executable(para_foreach_elt para_foreach_elt.cpp)
target_link_libraries(para_foreach_elt ${cgogn_core_LIBRARIES})


set_target_properties(para_foreach_elt PROPERTIES FOLDER examples/core)
