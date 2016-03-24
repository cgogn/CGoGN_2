project(cgogn_io
	LANGUAGES CXX
)

find_package(ZLIB)

set(HEADER_FILES
	surface_import.h
	volume_import.h
	map_import.h
	map_export.h
	import_ply_data.h
	io_utils.h
	data_io.h
	dll.h
	c_locale.h
	vtk_io.h
	off_io.h
	obj_io.h
	ply_io.h
	lm6_io.h
	mesh_io_gen.h
)

set(SOURCE_FILES
	surface_import.cpp
	volume_import.cpp
	import_ply_data.cpp
	mesh_io_gen.cpp
	io_utils.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_LM6_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_PLY_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${ZLIB_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include/cgogn/io>
)

if (${ZLIB_FOUND})
	target_compile_definitions(${PROJECT_NAME} PUBLIC -DZLIB_CONST)
	target_compile_definitions(${PROJECT_NAME} PUBLIC -DCGOGN_WITH_ZLIB)
endif()

target_link_libraries(${PROJECT_NAME} tinyxml2 cgogn_core cgogn_geometry ply lm6 ${ZLIB_LIBRARIES})

install(DIRECTORY .
	DESTINATION include/cgogn/io
	FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

add_subdirectory(mesh_generation)
add_subdirectory(examples)
