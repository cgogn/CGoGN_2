if (CGOGN_PROVIDE_EIGEN)
	add_subdirectory(eigen-3.2.7)
endif(CGOGN_PROVIDE_EIGEN)

if(CGOGN_BUILD_TESTS)
	add_subdirectory(gtest)
endif(CGOGN_BUILD_TESTS)