project(cgogn_rendering
	LANGUAGES CXX
)

find_package(Qt5 5.4.0 COMPONENTS Gui REQUIRED)
find_package(QOGLViewer REQUIRED)

add_library(${PROJECT_NAME} SHARED "")

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/dll.h"
		"${CMAKE_CURRENT_LIST_DIR}/drawer.h"
		"${CMAKE_CURRENT_LIST_DIR}/drawer.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/topo_drawer.h"
		"${CMAKE_CURRENT_LIST_DIR}/topo_drawer.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/volume_drawer.h"
		"${CMAKE_CURRENT_LIST_DIR}/volume_drawer.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/map_render.h"
		"${CMAKE_CURRENT_LIST_DIR}/map_render.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/wall_paper.h"
		"${CMAKE_CURRENT_LIST_DIR}/wall_paper.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/frame_manipulator.h"
		"${CMAKE_CURRENT_LIST_DIR}/frame_manipulator.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/text_drawer.h"
		"${CMAKE_CURRENT_LIST_DIR}/text_drawer.cpp"

		"${CMAKE_CURRENT_LIST_DIR}/shaders/vbo.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_program.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_program.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_simple_color.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_simple_color.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_color_per_vertex.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_color_per_vertex.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_scalar_per_vertex.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_scalar_per_vertex.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_flat.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_flat.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_vector_per_vertex.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_vector_per_vertex.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_phong.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_phong.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_bold_line.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_bold_line.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_round_point.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_round_point.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_point_sprite.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_point_sprite.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_explode_volumes.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_explode_volumes.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_explode_volumes_line.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_explode_volumes_line.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_texture.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_texture.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_text.h"
		"${CMAKE_CURRENT_LIST_DIR}/shaders/shader_text.cpp"
)

if (Qt5Gui_VERSION VERSION_GREATER 5.5.99)
	target_sources(${PROJECT_NAME}
		PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/transparency_shaders/shader_transparent_quad.h"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_shaders/shader_transparent_quad.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_shaders/shader_transparent_flat.h"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_shaders/shader_transparent_flat.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_shaders/shader_transparent_phong.h"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_shaders/shader_transparent_phong.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_shaders/shader_transparent_volumes.h"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_shaders/shader_transparent_volumes.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_shaders/shader_copy_depth.h"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_shaders/shader_copy_depth.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_drawer.h"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_drawer.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_volume_drawer.h"
		"${CMAKE_CURRENT_LIST_DIR}/transparency_volume_drawer.cpp"
	)
else()
	message(WARNING "Qt version < 5.6 : not compiling transparency rendering (need MRT FBO)")
endif()

qt5_add_resources(RCC_FILES fonte.qrc)

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_RENDERING_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_QOGLVIEWER_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
	$<INSTALL_INTERFACE:include/cgogn/thirdparty>
)

target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_geometry Qt5::Gui QOGLViewer)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR}/shaders transparency_shaders
	DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/cgogn/rendering
	FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CGOGN_SOURCE_DIR}" "include;include/cgogn/thirdparty")

# VisualStudio nice source organisation
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
