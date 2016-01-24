project(${CGOGN_TEST_PREFIX}mrcmap2
	LANGUAGES CXX
)

add_executable(mrcmap2 mrcmap2.cpp)
target_link_libraries(mrcmap2 cgogn_core)
