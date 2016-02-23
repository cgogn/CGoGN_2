project(cgogn_io
	LANGUAGES CXX C
	)

set(HEADER_FILES
	surface_import.h
	volume_import.h
	map_import.h
	map_export.h
	ply.h
	import_ply_data.h
	dll.h
	)

set(SOURCE_FILES
	surface_import.cpp
	volume_import.cpp
	ply.c
	import_ply_data.cpp
	)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${TinyXml2_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include/cgogn/io>
	)

target_link_libraries(${PROJECT_NAME} tinyxml2 cgogn_core cgogn_geometry)

install(DIRECTORY .
	DESTINATION include/cgogn/cgogn_core
	FILES_MATCHING PATTERN "*.h"
	)

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	)

add_subdirectory(examples)
