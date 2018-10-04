project(cgogn_io_test
	LANGUAGES CXX
)

find_package(cgogn_geometry REQUIRED)
find_package(cgogn_io REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/vtk_import_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/meshb_import_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/msh_import_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/obj_import_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/off_import_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/ply_import_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/tet_import_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/vtk_import_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/nastran_import_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/tetgen_import_test.cpp"
)

add_definitions("-DCGOGN_TEST_MESHES_PATH=${CMAKE_SOURCE_DIR}/data/meshes/")

target_link_libraries(${PROJECT_NAME} gtest cgogn::geometry cgogn::io)

add_test(NAME ${PROJECT_NAME} WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER tests)
