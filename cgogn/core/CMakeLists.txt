project(cgogn_core
	LANGUAGES CXX
)

set(HEADER_FILES
	basic/dll.h
	basic/dart.h
	basic/dart_marker.h
	basic/cell.h
	basic/cell_marker.h

	cmap/map_traits.h
	cmap/map_base_data.h
	cmap/map_base.h
	cmap/cmap0.h
	cmap/cmap1.h
	cmap/cmap2.h
	cmap/cmap2_builder.h
	cmap/cmap3.h
	cmap/cmap3_builder.h
	cmap/attribute_handler.h
	cmap/sanity_check.h

	container/chunk_array_container.h
	container/chunk_array_factory.h
	container/chunk_array_gen.h
	container/chunk_array.h
	container/chunk_stack.h

	utils/assert.h
	utils/buffers.h
	utils/definitions.h
	utils/dll.h
	utils/make_unique.h
	utils/name_types.h
	utils/serialization.h
	utils/thread.h
	utils/thread_pool.h
	utils/thread_barrier.h
	utils/string.h
	utils/precision.h
)

set(SOURCE_FILES
	basic/dart_marker.cpp

	cmap/map_base_data.cpp
        cmap/cmap0.cpp
        cmap/cmap1.cpp
        cmap/cmap2.cpp
	cmap/cmap3.cpp
	cmap/cmap2_builder.cpp
	cmap/cmap3_builder.cpp

	container/chunk_array_container.cpp
	container/chunk_array_gen.cpp
	container/chunk_array.cpp
	container/chunk_stack.cpp
	container/chunk_array_container.cpp
	container/chunk_array_factory.cpp

	utils/assert.cpp
	utils/name_types.cpp
	utils/thread.cpp
	utils/thread_pool.cpp
	utils/serialization.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

# use of target_compile_options to have a transitive c++11 flag
if(NOT MSVC)
	target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include/cgogn/core>
)

 install(DIRECTORY basic cmap container utils
	DESTINATION include/cgogn/core/
	FILES_MATCHING PATTERN "*.h"
 )

 install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
 )

add_subdirectory(examples)

if(CGOGN_BUILD_TESTS)
	add_subdirectory(tests)
endif()
