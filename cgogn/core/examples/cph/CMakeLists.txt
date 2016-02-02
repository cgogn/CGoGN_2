project(${CGOGN_TEST_PREFIX}map
	LANGUAGES CXX
)

set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_executable(cph2 cph2.cpp)
target_link_libraries(cph2 cgogn_core cgogn_io)
