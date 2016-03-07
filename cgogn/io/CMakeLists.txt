project(cgogn_io
	LANGUAGES CXX
)

find_package(ZLIB)
add_definitions("-DZLIB_CONST")

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
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_EIGEN3_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_PLY_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${ZLIB_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include/cgogn/io>
)

target_link_libraries(${PROJECT_NAME} tinyxml2 cgogn_core cgogn_geometry ply ${ZLIB_LIBRARIES})

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

add_subdirectory(examples)
