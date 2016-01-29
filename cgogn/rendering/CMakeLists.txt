project(cgogn_rendering
	LANGUAGES CXX
	)

find_package(Qt5Widgets)
#find_package(Qt5OpenGL)


set(HEADER_FILES
	dll.h
	map_render.h
	shaders/vbo.h
	shaders/shader_program.h
	shaders/shader_simple_color.h
	shaders/shader_color_per_vertex.h
	shaders/shader_flat.h
	shaders/shader_vector_per_vertex.h
	)

set(SOURCE_FILES
	shaders/shader_program.cpp
	shaders/shader_simple_color.cpp
	shaders/shader_color_per_vertex.cpp
	shaders/shader_flat.cpp
	shaders/shader_vector_per_vertex.cpp

	)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${QOGLViewer_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${TinyXml2_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include/cgogn/rendering>
	)


target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_geometry Qt5::Widgets)

install(DIRECTORY .
	DESTINATION include/cgogn/rendering
	FILES_MATCHING PATTERN "*.h"
	)

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	)

add_subdirectory(examples)

#if(CGOGN_BUILD_TESTS)
#	add_subdirectory(tests)
#endif()


