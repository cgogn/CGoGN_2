project(${CGOGN_TEST_PREFIX}chunk_array
		LANGUAGES CXX
	   )


add_executable(test_chunk_array test_chunk_array.cpp)
target_link_libraries(test_chunk_array cgogn_core)

add_executable(bench_chunk_array bench_chunk_array.cpp)
target_link_libraries(bench_chunk_array cgogn_core)


install(TARGETS test_chunk_array
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "bin/test"
           )

install(TARGETS bench_chunk_array
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "bin/bench"
        )
