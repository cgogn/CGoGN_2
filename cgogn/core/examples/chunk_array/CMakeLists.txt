project(${CGOGN_TEST_PREFIX}chunk_array
		LANGUAGES CXX
	   )

#get_property(cgogn_core_includes
#			  TARGET cgogn_core
#			  PROPERTY INCLUDE_DIRECTORIES
#			)

add_executable(chunk_array chunk_array.cpp)
target_link_libraries(chunk_array cgogn_core)
#target_include_directories(test_chunk_array PRIVATE ${cgogn_core_includes})

add_executable(bench_chunk_array bench_chunk_array.cpp)
target_link_libraries(bench_chunk_array cgogn_core)
#target_include_directories(bench_chunk_array PRIVATE ${cgogn_core_includes})

add_executable(chunk_array2 chunk_array2.cpp)
target_link_libraries(chunk_array2 cgogn_core)


install(TARGETS chunk_array
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "bin/test"
           )

install(TARGETS bench_chunk_array
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "bin/bench"
        )
