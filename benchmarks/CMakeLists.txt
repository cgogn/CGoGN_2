set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_subdirectory(multithreading)
add_subdirectory(tri_map)
add_subdirectory(quad_map)
add_subdirectory(tetra_map)
add_subdirectory(comparison)
