project(${CGOGN_TEST_PREFIX}chunk_array
		LANGUAGES CXX
	   )

#set(HEADER_FILES
#	)

#set(SOURCE_FILES
#	test_chunk_array.cpp
#	)

#add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})

#target_link_libraries(${PROJECT_NAME} cgogn_core)

add_executable(test_chunk_array test_chunk_array.cpp)
target_link_libraries(test_chunk_array cgogn_core)

add_executable(bench_chunk_array bench_chunk_array.cpp)
target_link_libraries(bench_chunk_array cgogn_core)




#install(TARGETS ${PROJECT_NAME}
#        EXPORT ${PROJECT_NAME}Targets
#        RUNTIME DESTINATION "bin/test"
#	   )
