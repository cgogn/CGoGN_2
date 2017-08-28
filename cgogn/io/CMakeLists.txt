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
	graph_import.h
	graph_export.h
	import_ply_data.h
	io_utils.h
	data_io.h
	dll.h
	c_locale.h
	mesh_io_gen.h
	surface_export.h
	volume_export.hformats/
	formats/vtk_io.h
	formats/off_io.h
	formats/obj_io.h
	formats/2dm_io.h
	formats/ply_io.h
	formats/lm6_io.h
	formats/stl_io.h
	formats/io_cg.h
	formats/io_cskel.h
	formats/io_dot.h
	formats/io_obj.h
	formats/io_skel.h
	formats/io_vtk.h
	formats/msh_io.h
	formats/tetgen_io.h
	formats/nastran_io.h
	formats/tet_io.h
	formats/tetmesh_io.h
)

set(SOURCE_FILES
	map_export.cpp
	map_import.cpp
	graph_export.cpp
	graph_import.cpp
	surface_import.cpp
	surface_export.cpp
	volume_import.cpp
	volume_export.cpp
	import_ply_data.cpp
	mesh_io_gen.cpp
	io_utils.cpp
	formats/vtk_io.cpp
	formats/lm6_io.cpp
	formats/msh_io.cpp
	formats/obj_io.cpp
	formats/2dm_io.cpp
	formats/off_io.cpp
	formats/ply_io.cpp
	formats/stl_io.cpp
	formats/tetgen_io.cpp
	formats/nastran_io.cpp
	formats/tet_io.cpp
	formats/tetmesh_io.cpp
	formats/io_cg.cpp
	formats/io_cskel.cpp
#	formats/io_dot.cpp
	formats/io_obj.cpp
	formats/io_skel.cpp
	formats/io_vtk.cpp
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
