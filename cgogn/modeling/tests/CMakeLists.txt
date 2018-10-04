project(cgogn_modeling_test
	LANGUAGES CXX
)

find_package(cgogn_geometry REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(cgogn_modeling REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/algos/catmull_clark_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/algos/dual_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/tiling/square_tiling_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/tiling/triangular_tiling_test.cpp"
)

add_definitions("-DCGOGN_TEST_MESHES_PATH=${CMAKE_SOURCE_DIR}/data/meshes/")

target_link_libraries(${PROJECT_NAME} gtest cgogn::geometry cgogn::io cgogn::modeling)

add_test(NAME ${PROJECT_NAME} WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER tests)
