project(cgogn_core
	LANGUAGES CXX
)

set(HEADER_FILES
	src/core/basic/dll.h
	src/core/basic/dart.h
	src/core/basic/dart_marker.h
	src/core/basic/cell.h
	src/core/basic/cell_marker.h

	src/core/container/chunk_array_container.h
	src/core/container/chunk_array_factory.h
	src/core/container/chunk_array_gen.h
	src/core/container/chunk_array.h
	src/core/container/chunk_stack.h

	src/core/cmap/map_base_data.h
	src/core/cmap/map_base.h
	src/core/cmap/cmap0.h
	src/core/cmap/cmap1.h
	src/core/cmap/cmap2.h
	src/core/cmap/cmap2_builder.h
	src/core/cmap/cmap3.h
	src/core/cmap/attribute_handler.h

	src/utils/assert.h
	src/utils/buffers.h
	src/utils/definitions.h
	src/utils/dll.h
	src/utils/make_unique.h
	src/utils/name_types.h
	src/utils/serialization.h
	src/utils/thread.h
)

set(SOURCE_FILES
#	src/core/basic/cell_marker.cpp
	src/core/basic/dart_marker.cpp

	src/core/container/chunk_array_container.cpp

	src/core/cmap/map_base_data.cpp

	src/utils/assert.cpp
	src/utils/thread.cpp
	src/utils/name_types.cpp
	src/utils/serialization.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

# target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:include/cgogn/cgogn_core>
)

add_subdirectory(examples)

# target_link_libraries(${PROJECT_NAME} cgogn_utils)

# install(DIRECTORY basic container map
# 	DESTINATION include/cgogn/cgogn_core
# 	FILES_MATCHING PATTERN "*.h"
# )

# install(TARGETS ${PROJECT_NAME}
# 	EXPORT ${PROJECT_NAME}Targets
# 	RUNTIME DESTINATION bin
# 	LIBRARY DESTINATION lib
# 	ARCHIVE DESTINATION lib
# )
