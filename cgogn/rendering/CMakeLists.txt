project(cgogn_rendering
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(Qt5Widgets)

set(HEADER_FILES
	dll.h
	map_render.h
	shaders/vbo.h
	shaders/shader_program.h
	shaders/shader_simple_color.h
	shaders/shader_color_per_vertex.h
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
	topo_render.h
	volume_render.h
	wall_paper.h
)

set(SOURCE_FILES
	shaders/shader_program.cpp
	shaders/shader_simple_color.cpp
	shaders/shader_color_per_vertex.cpp
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
	topo_render.cpp
	volume_render.cpp
	map_render.cpp
	wall_paper.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_QOGLVIEWER_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_geometry_LIBRARIES} Qt5::Widgets)

file(GLOB HEADERS "." "*.h")
install(FILES ${HEADERS}
	DESTINATION include/cgogn/rendering)
install(DIRECTORY shaders
	DESTINATION include/cgogn/rendering
	FILES_MATCHING PATTERN "*.h"
)



######## 1. Build tree

export(
	TARGETS ${PROJECT_NAME}
	FILE "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Targets.cmake")

set(CGOGN_RENDERING_INCLUDE_DIRS "${CGOGN_SOURCE_DIR}")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake"
	PATH_VARS CGOGN_RENDERING_INCLUDE_DIRS
	INSTALL_DESTINATION "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}"
)

write_basic_package_version_file(
	"${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake"
	VERSION ${CGOGN_VERSION_MAJOR}.${CGOGN_VERSION_MINOR}.${CGOGN_VERSION_PATCH}
	COMPATIBILITY ExactVersion
)

######## 2. Install tree

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION "bin"
	LIBRARY DESTINATION "lib"
	ARCHIVE DESTINATION "lib"
)
install(EXPORT ${PROJECT_NAME}Targets DESTINATION "lib/cmake/${PROJECT_NAME}")

## <package_name>ConfigVersion.cmake
write_basic_package_version_file(
	"${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake"
	VERSION ${CGOGN_VERSION_MAJOR}.${CGOGN_VERSION_MINOR}.${CGOGN_VERSION_PATCH}
	COMPATIBILITY ExactVersion
)

## <package_name>Config.cmake
set(CGOGN_RENDERING_INCLUDE_DIRS "include")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake"
	PATH_VARS CGOGN_RENDERING_INCLUDE_DIRS
	INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
)

install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}")
install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}" RENAME "${PROJECT_NAME}Config.cmake")


add_subdirectory(examples)
