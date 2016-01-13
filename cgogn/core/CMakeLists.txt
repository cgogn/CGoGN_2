project(cgogn_core
	LANGUAGES CXX
)

set(HEADER_FILES
	basic/dll.h
	basic/dart.h
	basic/dart_marker.h
	basic/cell.h
	basic/cell_marker.h

	container/chunk_array_container.h
	container/chunk_array_factory.h
	container/chunk_array_gen.h
	container/chunk_array.h
	container/chunk_stack.h

	map/map_base_data.h
	map/map_base.h
	map/cmap1.h
	map/cmap2.h
	map/cmap2_builder.h
	map/cmap3.h
	map/cmap_tri.h
	map/attribute_handler.h

	mr_map/mr_map_base_data.h
	mr_map/mr_map_base.h
	mr_map/mr_map1.h
	mr_map/mr_attribute_handler.h
)

set(SOURCE_FILES
#	basic/cell_marker.cpp
	basic/dart_marker.cpp

	container/chunk_array_container.cpp

	map/map_base_data.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include/cgogn/cgogn_core>
)

target_link_libraries(${PROJECT_NAME} cgogn_utils)

install(DIRECTORY basic container map
	DESTINATION include/cgogn/cgogn_core
	FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)
