project(cgogn_geometry_test
	LANGUAGES CXX
)

set(SOURCE_FILES
	types/vec_test.cpp
	types/plane_3d_test.cpp
	types/bounding_box_test.cpp
	main.cpp
)


add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(cgogn_geometry_test gtest cgogn_geometry)

target_include_directories(cgogn_geometry_test PRIVATE ${CMAKE_SOURCE_DIR}/thirdparty/googletest-master/googletest/include)
link_directories(${CMAKE_SOURCE_DIR}/thirdparty/googletest-master/googletest/lib)

add_test(NAME "${PROJECT_NAME}" WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" COMMAND ${PROJECT_NAME})
