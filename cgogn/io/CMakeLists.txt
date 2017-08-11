project(cgogn_io
	LANGUAGES CXX
)

find_package(ply REQUIRED)
find_package(lm6 REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(Zlib REQUIRED NO_CMAKE_ENVIRONMENT_PATH)

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
	2dm_io.h
	ply_io.h
	lm6_io.h
	stl_io.h
	mesh_io_gen.h
	msh_io.h
	tetgen_io.h
	nastran_io.h
	tet_io.h
	tetmesh_io.h
	surface_export.h
	volume_export.h
)

set(SOURCE_FILES
	map_export.cpp
	map_import.cpp
	surface_import.cpp
	surface_export.cpp
	volume_import.cpp
	volume_export.cpp
	import_ply_data.cpp
	mesh_io_gen.cpp
	io_utils.cpp
	vtk_io.cpp
	lm6_io.cpp
	msh_io.cpp
	obj_io.cpp
	2dm_io.cpp
	off_io.cpp
	ply_io.cpp
	stl_io.cpp
	tetgen_io.cpp
	nastran_io.cpp
	tet_io.cpp
	tetmesh_io.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_IO_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_LM6_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_PLY_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_ZLIB_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
	$<INSTALL_INTERFACE:include/cgogn/thirdparty>
)

target_link_libraries(${PROJECT_NAME} ${Zlib_LIBRARIES} cgogn_core cgogn_geometry ${ply_LIBRARIES} ${lm6_LIBRARIES} ${tinyxml2_LIBRARIES})

file(GLOB HEADERS "." "*.h")
install(FILES ${HEADERS}
	DESTINATION include/cgogn/io)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CGOGN_SOURCE_DIR}" "include;include/cgogn/thirdparty")

# VisualStudio nice source organisation
source_group("IO" FILES ${HEADER_FILES} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
