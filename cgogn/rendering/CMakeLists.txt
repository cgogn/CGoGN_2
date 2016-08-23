project(cgogn_rendering
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(HEADER_FILES
	dll.h
	shaders/vbo.h
	shaders/shader_program.h
	shaders/shader_simple_color.h
	shaders/shader_color_per_vertex.h
	shaders/shader_scalar_per_vertex.h
	shaders/shader_flat.h
	shaders/shader_vector_per_vertex.h
	shaders/shader_phong.h
	shaders/shader_bold_line.h
	shaders/shader_round_point.h
	shaders/shader_point_sprite.h
	shaders/shader_explode_volumes.h
	shaders/shader_explode_volumes_line.h
	shaders/shader_texture.h
	drawer.h
	topo_drawer.h
	volume_drawer.h
	map_render.h
	wall_paper.h
	frame_manipulator.h
)

set(SOURCE_FILES
	shaders/shader_program.cpp
	shaders/shader_simple_color.cpp
	shaders/shader_color_per_vertex.cpp
	shaders/shader_scalar_per_vertex.cpp
	shaders/shader_flat.cpp
	shaders/shader_vector_per_vertex.cpp
	shaders/shader_phong.cpp
	shaders/shader_bold_line.cpp
	shaders/shader_round_point.cpp
	shaders/shader_point_sprite.cpp
	shaders/shader_explode_volumes.cpp
	shaders/shader_explode_volumes_line.cpp
	shaders/shader_texture.cpp
	drawer.cpp
	topo_drawer.cpp
	volume_drawer.cpp
	map_render.cpp
	wall_paper.cpp
	frame_manipulator.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_RENDERING_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_QOGLVIEWER_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_geometry_LIBRARIES} ${Qt5Widgets_LIBRARIES})

file(GLOB HEADERS "." "*.h")
install(FILES ${HEADERS}
	DESTINATION include/cgogn/rendering)
install(DIRECTORY shaders
	DESTINATION include/cgogn/rendering
	FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CGOGN_SOURCE_DIR}" "include")
