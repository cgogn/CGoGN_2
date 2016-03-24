project(cgogn_modelisation_test
	LANGUAGES CXX
)

set(SOURCE_FILES
	algos/catmull_clark_test.cpp
	main.cpp
)

add_definitions("-DCGOGN_TEST_MESHES_PATH=${CMAKE_SOURCE_DIR}/data/meshes/")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(cgogn_modelisation_test gtest cgogn_geometry cgogn_io)

target_include_directories(cgogn_modelisation_test PRIVATE ${CMAKE_SOURCE_DIR}/thirdparty/googletest-master/googletest/include)
link_directories(${CMAKE_SOURCE_DIR}/thirdparty/googletest-master/googletest/lib)

add_test(NAME "${PROJECT_NAME}" WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" COMMAND ${PROJECT_NAME})
