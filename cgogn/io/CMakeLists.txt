project(cgogn_io
	LANGUAGES CXX
)

find_package(ply REQUIRED)
find_package(Meshb REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(ZLIB REQUIRED)

add_library(${PROJECT_NAME} SHARED "")

add_library(cgogn::io ALIAS cgogn_io)

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/dll.h"
		"${CMAKE_CURRENT_LIST_DIR}/data_io.h"
		"${CMAKE_CURRENT_LIST_DIR}/surface_import.h"
		"${CMAKE_CURRENT_LIST_DIR}/volume_import.h"
		"${CMAKE_CURRENT_LIST_DIR}/map_import.h"
		"${CMAKE_CURRENT_LIST_DIR}/map_export.h"
		"${CMAKE_CURRENT_LIST_DIR}/graph_import.h"
		"${CMAKE_CURRENT_LIST_DIR}/graph_import.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/graph_export.h"
		"${CMAKE_CURRENT_LIST_DIR}/io_utils.h"
		"${CMAKE_CURRENT_LIST_DIR}/io_utils.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/dll.h"
		"${CMAKE_CURRENT_LIST_DIR}/c_locale.h"
		"${CMAKE_CURRENT_LIST_DIR}/mesh_io_gen.h"
		"${CMAKE_CURRENT_LIST_DIR}/mesh_io_gen.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/surface_export.h"
		"${CMAKE_CURRENT_LIST_DIR}/volume_export.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/2dm.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/dot.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/cg.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/cskel.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/meshb.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/msh.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/nastran.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/obj.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/off.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/ply.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/ply.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/formats/ply_data.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/ply_data.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/formats/skel.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/stl.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/tet.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/tetgen.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/tetmesh.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/vtk.h"
		"${CMAKE_CURRENT_LIST_DIR}/formats/vtk.cpp"
)

if(${CGOGN_EXTERNAL_TEMPLATES})
	target_sources(${PROJECT_NAME}
		PRIVATE
			"${CMAKE_CURRENT_LIST_DIR}/external_templates.cpp"
	)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
										DEBUG_POSTFIX "_d"
										EXPORT_NAME io)

target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_IO_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/cgogn/thirdparty>
)

target_link_libraries(${PROJECT_NAME} cgogn::core cgogn::geometry ZLIB::ZLIB ply Meshb TinyXML2::TinyXML2)

set(PKG_CONFIG_REQUIRES "cgogn_core cgogn_geometry")
configure_file(${PROJECT_SOURCE_DIR}/cgogn_io.pc.in ${CMAKE_CURRENT_BINARY_DIR}/cgogn_io.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cgogn_io.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/io
	FILES_MATCHING PATTERN "*.h"
	REGEX "(examples|tests)" EXCLUDE
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}")

# VisualStudio nice source organisation
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
