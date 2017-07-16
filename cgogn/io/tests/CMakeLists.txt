project(cgogn_io_test
	LANGUAGES CXX
)

find_package(cgogn_geometry REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(ply REQUIRED)
find_package(lm6 REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(Zlib REQUIRED NO_CMAKE_ENVIRONMENT_PATH)

set(SOURCE_FILES
	main.cpp
	vtk_import_test.cpp
	medit_import_test.cpp
	msh_import_test.cpp
	obj_import_test.cpp
	off_import_test.cpp
	ply_import_test.cpp
	tet_import_test.cpp
	vtk_import_test.cpp
	nastran_import_test.cpp
	tetgen_import_test.cpp
)

add_definitions("-DCGOGN_TEST_MESHES_PATH=${CMAKE_SOURCE_DIR}/data/meshes/")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} gtest ${cgogn_geometry_LIBRARIES} ${cgogn_io_LIBRARIES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/thirdparty/googletest-master/googletest/include)
link_directories(${CMAKE_SOURCE_DIR}/thirdparty/googletest-master/googletest/lib)

add_test(NAME "${PROJECT_NAME}" WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" COMMAND ${PROJECT_NAME})


set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER tests)