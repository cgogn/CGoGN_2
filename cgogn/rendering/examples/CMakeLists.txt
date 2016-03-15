cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_rendering_examples
	LANGUAGES CXX
)

set(CMAKE_AUTOMOC ON)


set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")


add_executable(simple_viewer simple_viewer.cpp)
target_link_libraries(simple_viewer cgogn_core cgogn_io cgogn_rendering QOGLViewer)

add_executable(viewer_per_face viewer_per_face.cpp)
target_link_libraries(viewer_per_face cgogn_core cgogn_io cgogn_rendering QOGLViewer)

add_executable(viewer_topo viewer_topo.cpp)
target_link_libraries(viewer_topo cgogn_core cgogn_io cgogn_rendering QOGLViewer)

add_executable(drawing drawing.cpp)
target_link_libraries(drawing cgogn_rendering QOGLViewer)

add_executable(picking_viewer picking_viewer.cpp)
target_link_libraries(picking_viewer cgogn_core cgogn_io cgogn_rendering QOGLViewer)


add_executable(viewer_topo3 viewer_topo3.cpp)
target_link_libraries(viewer_topo3 cgogn_core cgogn_io cgogn_rendering QOGLViewer)


