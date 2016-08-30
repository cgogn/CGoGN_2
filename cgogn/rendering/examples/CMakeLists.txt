cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_rendering_examples
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(cgogn_rendering REQUIRED)
find_package(cgogn_modeling REQUIRED)
find_package(QOGLViewer REQUIRED)

set(CMAKE_AUTOMOC ON)


set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")


add_executable(simple_viewer simple_viewer.cpp)
target_link_libraries(simple_viewer ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} ${QOGLViewer_LIBRARIES})

add_executable(viewer_per_face viewer_per_face.cpp)
target_link_libraries(viewer_per_face ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} ${QOGLViewer_LIBRARIES})

add_executable(viewer_topo viewer_topo.cpp)
target_link_libraries(viewer_topo ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} ${cgogn_modeling_LIBRARIES} ${QOGLViewer_LIBRARIES})

add_executable(drawing drawing.cpp)
target_link_libraries(drawing ${cgogn_rendering_LIBRARIES} ${QOGLViewer_LIBRARIES})

add_executable(picking_viewer picking_viewer.cpp)
target_link_libraries(picking_viewer ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} ${QOGLViewer_LIBRARIES})

add_executable(viewer_topo3 viewer_topo3.cpp)
target_link_libraries(viewer_topo3 ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} ${QOGLViewer_LIBRARIES})
