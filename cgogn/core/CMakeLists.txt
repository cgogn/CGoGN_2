project(cgogn_core
	LANGUAGES CXX
)

set(HEADER_FILES
	dll.h

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
	cmap/attribute.h
	cmap/cmap2_tri.h
	cmap/cmap2_tri_builder.h
	cmap/cmap2_quad.h
	cmap/cmap2_quad_builder.h
	cmap/cmap3_tetra.h
	cmap/cmap3_tetra_builder.h
	cmap/cmap3_hexa.h
	cmap/cmap3_hexa_builder.h

	container/chunk_array_container.h
	container/chunk_array_factory.h
	container/chunk_array_gen.h
	container/chunk_array.h
	container/chunk_stack.h

	utils/assert.h
	utils/buffers.h
	utils/definitions.h
	utils/endian.h
	utils/unique_ptr.h
	utils/name_types.h
	utils/serialization.h
	utils/thread.h
	utils/thread_pool.h
	utils/string.h
	utils/masks.h
	utils/logger.h
	utils/log_entry.h
	utils/logger_output.h
	utils/log_stream.h
	utils/numerics.h
	utils/type_traits.h
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
	cmap/cmap2_tri.cpp
	cmap/cmap2_tri_builder.cpp
	cmap/cmap2_quad.cpp
	cmap/cmap2_quad_builder.cpp
	cmap/cmap3_tetra.cpp
	cmap/cmap3_tetra_builder.cpp
	cmap/cmap3_hexa.cpp
	cmap/cmap3_hexa_builder.cpp

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
	utils/logger.cpp
	utils/log_entry.cpp
	utils/logger_output.cpp
	utils/log_stream.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
# generate_export_header(${PROJECT_NAME}
#   EXPORT_MACRO_NAME "CGOGN_CORE_API"
#   EXPORT_FILE_NAME "${CMAKE_BINARY_DIR}/exports/${PROJECT_NAME}_export.h"
# )

# use of target_compile_options to have transitive flags
if(NOT MSVC)
	target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
	remove_flags(CMAKE_CXX_FLAGS "-std=c++11")
	if(${CGOGN_USE_CXX11_ABI})
		target_compile_options(${PROJECT_NAME} PUBLIC "-D_GLIBCXX_USE_CXX11_ABI")
		remove_flags(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI")
	endif()
	if (${CGOGN_USE_GLIBCXX_DEBUG})
		target_compile_options(${PROJECT_NAME} PUBLIC "-D_GLIBCXX_DEBUG")
		remove_flags(CMAKE_CXX_FLAGS "-D_GLIBCXX_DEBUG")
		if(${CGOGN_USE_GLIBCXX_DEBUG_PEDANTIC})
			target_compile_options(${PROJECT_NAME} PUBLIC "-D_GLIBCXX_DEBUG_PEDANTIC")
			remove_flags(CMAKE_CXX_FLAGS "-D_GLIBCXX_DEBUG_PEDANTIC")
		endif()
	endif()
	if(${CGOGN_USE_PARALLEL_GLIBCXX} AND (NOT ${CGOGN_USE_GLIBCXX_DEBUG}))
		target_compile_options(${PROJECT_NAME} PUBLIC "-D_GLIBCXX_PARALLEL")
		remove_flags(CMAKE_CXX_FLAGS "-D_GLIBCXX_PARALLEL")
	endif()
endif()

if(MSVC)
	target_compile_options(${PROJECT_NAME} PUBLIC "-D_USE_MATH_DEFINES")
	## CGOGN_WIN_VER : has value 61 for windows 7, 62 for windows 8, 63 for windows 8.1, 100 for windows 10
	set(WIN_VERSION "")
	string(REPLACE "." "" WIN_VERSION ${CMAKE_SYSTEM_VERSION})
	target_compile_options(${PROJECT_NAME} PUBLIC "-DCGOGN_WIN_VER=${WIN_VERSION}")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TERMCOLOR_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
)


target_compile_options(${PROJECT_NAME} PUBLIC "-DCGOGN_LITTLE_ENDIAN=1234")
target_compile_options(${PROJECT_NAME} PUBLIC "-DCGOGN_BIG_ENDIAN=4321")
if(CGOGN_TEST_BIG_ENDIAN)
	target_compile_options(${PROJECT_NAME} PUBLIC "-DCGOGN_ENDIANNESS=CGOGN_BIG_ENDIAN")
else()
	target_compile_options(${PROJECT_NAME} PUBLIC "-DCGOGN_ENDIANNESS=CGOGN_LITTLE_ENDIAN")
endif()

install(FILES "dll.h" DESTINATION "include/cgogn/core/")
install(DIRECTORY basic cmap container utils
	DESTINATION "include/cgogn/core/"
	FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CGOGN_SOURCE_DIR};${CGOGN_THIRDPARTY_TERMCOLOR_INCLUDE_DIR}" "include")
