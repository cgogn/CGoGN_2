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

add_executable(mt_viewer mt_viewer.cpp)
target_link_libraries(mt_viewer ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} ${QOGLViewer_LIBRARIES})

set_target_properties(simple_viewer viewer_per_face viewer_topo drawing picking_viewer viewer_topo3 mt_viewer
PROPERTIES FOLDER examples/rendering)


if (Qt5Widgets_VERSION VERSION_GREATER 5.5.99)

	add_executable(transparency_viewer transparency_viewer.cpp)
	target_link_libraries(transparency_viewer ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} ${QOGLViewer_LIBRARIES})

	add_executable(transparencies_viewer transparencies_viewer.cpp)
	target_link_libraries(transparencies_viewer ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} ${QOGLViewer_LIBRARIES})

	add_executable(transparency_volume_viewer transparency_volume_viewer.cpp)
	target_link_libraries(transparency_volume_viewer ${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} ${QOGLViewer_LIBRARIES})

	set_target_properties(transparency_viewer transparency_volume_viewer transparencies_viewer 
	PROPERTIES FOLDER examples/rendering)

endif()
