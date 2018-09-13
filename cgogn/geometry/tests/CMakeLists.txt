project(cgogn_geometry_test
	LANGUAGES CXX
)

find_package(cgogn_geometry REQUIRED)
find_package(cgogn_io REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/types/vec_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/types/plane_3d_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/types/aabb_test.cpp"

		"${CMAKE_CURRENT_LIST_DIR}/functions/area_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/functions/normal_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/functions/distance_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/functions/intersection_test.cpp"

		"${CMAKE_CURRENT_LIST_DIR}/algos/algos_test.cpp"
)

add_definitions("-DCGOGN_TEST_MESHES_PATH=${CMAKE_SOURCE_DIR}/data/meshes/")

target_link_libraries(cgogn_geometry_test gtest cgogn::geometry cgogn::io)

add_test(NAME ${PROJECT_NAME} WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER tests)
