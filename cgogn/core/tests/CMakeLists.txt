project(cgogn_core_test
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/basic/dart_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/basic/cell_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/basic/dart_marker_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/basic/cell_marker_test.cpp"

		"${CMAKE_CURRENT_LIST_DIR}/container/chunk_array_container_test.cpp"

		"${CMAKE_CURRENT_LIST_DIR}/cmap/mapbase_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap0_topo_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap0_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap1_topo_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap1_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap2_topo_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap2_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap3_topo_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap3_test.cpp"

		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap2tri_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap2quad_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap3tetra_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap3hexa_test.cpp"

		"${CMAKE_CURRENT_LIST_DIR}/utils/endian_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/name_types_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/string_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/type_traits_test.cpp"
)

target_link_libraries(cgogn_core_test gtest gmock cgogn::core)

add_test(NAME ${PROJECT_NAME} WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER tests)
