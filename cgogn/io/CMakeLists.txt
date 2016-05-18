project(cgogn_io
	LANGUAGES CXX
)

find_package(ZLIB)
find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(ply REQUIRED)
find_package(lm6 REQUIRED)
find_package(tinyxml2 REQUIRED)

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
	msh_io.h
	tetgen_io.h
	nastran_io.h
	tet_io.h
)

set(SOURCE_FILES
	surface_import.cpp
	volume_import.cpp
	import_ply_data.cpp
	mesh_io_gen.cpp
	io_utils.cpp
	vtk_io.cpp
	lm6_io.cpp
	data_io.cpp
	msh_io.cpp
	obj_io.cpp
	off_io.cpp
	ply_io.cpp
	tetgen_io.cpp
	nastran_io.cpp
	tet_io.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_LM6_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_PLY_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${ZLIB_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>
)

if (${ZLIB_FOUND})
	target_compile_definitions(${PROJECT_NAME} PUBLIC "-DZLIB_CONST")
	target_compile_definitions(${PROJECT_NAME} PUBLIC "-DCGOGN_WITH_ZLIB")
        target_link_libraries(${PROJECT_NAME} ${ZLIB_LIBRARIES})
endif()

target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_geometry_LIBRARIES} ${ply_LIBRARIES} ${lm6_LIBRARIES} ${tinyxml2_LIBRARIES})

file(GLOB HEADERS "." "*.h")
install(FILES ${HEADERS}
	DESTINATION include/cgogn/io)

cgogn_create_package("${CGOGN_SOURCE_DIR}" "include")

add_subdirectory(mesh_generation)
