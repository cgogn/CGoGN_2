project(cgogn_io
	LANGUAGES CXX
)

find_package(ply REQUIRED)
find_package(Meshb REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(Zlib REQUIRED NO_CMAKE_ENVIRONMENT_PATH)

set(HEADER_FILES
	data_io.h
	surface_import.h
	volume_import.h
	map_import.h
	map_export.h
	graph_import.h
	graph_export.h
	io_utils.h
	dll.h
	c_locale.h
	mesh_io_gen.h
	surface_export.h
	volume_export.h
	formats/2dm.h
	formats/dot.h
	formats/cg.h
	formats/cskel.h
	formats/meshb.h
	formats/msh.h
	formats/nastran.h
	formats/obj.h
	formats/off.h
	formats/ply.h
	formats/ply_data.h
	formats/skel.h
	formats/stl.h
	formats/tet.h
	formats/tetgen.h
	formats/tetmesh.h
	formats/vtk.h
)

set(SOURCE_FILES
	graph_import.cpp
	mesh_io_gen.cpp
	io_utils.cpp
	formats/ply.cpp
	formats/ply_data.cpp
	formats/vtk.cpp
)

if(${CGOGN_EXTERNAL_TEMPLATES})
    set(SOURCE_FILES ${SOURCE_FILES} external_templates.cpp)
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_IO_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_MESHB_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_PLY_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_ZLIB_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
	$<INSTALL_INTERFACE:include/cgogn/thirdparty>
)

target_link_libraries(${PROJECT_NAME} ${Zlib_LIBRARIES} cgogn_core cgogn_geometry ${ply_LIBRARIES} ${Meshb_LIBRARIES} ${tinyxml2_LIBRARIES})

file(GLOB HEADERS "." "*.h")
install(FILES ${HEADERS}
	DESTINATION include/cgogn/io)
install(DIRECTORY formats
	DESTINATION include/cgogn/io/	FILES_MATCHING PATTERN "*.h")

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CGOGN_SOURCE_DIR}" "include;include/cgogn/thirdparty")

# VisualStudio nice source organisation
source_group("IO" FILES ${HEADER_FILES} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
