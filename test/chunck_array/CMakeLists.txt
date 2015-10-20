project(${CGOGN_TEST_PREFIX}chunck_array
		LANGUAGES CXX
	   )

set(HEADER_FILES
	)

set(SOURCE_FILES
	test_chunk_array.cpp
	)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} cgogn_core)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "bin/test"
	   )