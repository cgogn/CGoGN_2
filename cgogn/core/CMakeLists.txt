project(cgogn_core
	LANGUAGES CXX
)

add_library(${PROJECT_NAME} SHARED "")

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/dll.h"
		"${CMAKE_CURRENT_LIST_DIR}/basic/dart.h"
		"${CMAKE_CURRENT_LIST_DIR}/basic/dart_marker.h"
		"${CMAKE_CURRENT_LIST_DIR}/basic/cell.h"
		"${CMAKE_CURRENT_LIST_DIR}/basic/cell_marker.h"

		"${CMAKE_CURRENT_LIST_DIR}/cmap/map_traits.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/map_base_data.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/map_base_data.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/map_base.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap0.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap1.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap2.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap2_builder.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap3.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap3_builder.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/attribute.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap2_tri.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap2_quad.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap3_tetra.h"
		"${CMAKE_CURRENT_LIST_DIR}/cmap/cmap3_hexa.h"

		"${CMAKE_CURRENT_LIST_DIR}/container/chunk_array_container.h"
		"${CMAKE_CURRENT_LIST_DIR}/container/chunk_array_factory.h"
		"${CMAKE_CURRENT_LIST_DIR}/container/chunk_array_gen.h"
		"${CMAKE_CURRENT_LIST_DIR}/container/chunk_array.h"
		"${CMAKE_CURRENT_LIST_DIR}/container/chunk_stack.h"

		"${CMAKE_CURRENT_LIST_DIR}/graph/undirected_graph.h"
		"${CMAKE_CURRENT_LIST_DIR}/graph/undirected_graph_builder.h"

		"${CMAKE_CURRENT_LIST_DIR}/utils/assert.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/assert.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/buffers.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/definitions.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/endian.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/unique_ptr.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/name_types.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/name_types.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/serialization.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/serialization.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/thread.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/thread.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/thread_pool.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/thread_pool.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/string.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/string.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/masks.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/masks.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/logger.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/logger.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/log_entry.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/log_entry.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/logger_output.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/logger_output.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/log_stream.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/log_stream.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/numerics.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/type_traits.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/timer.h"
		"${CMAKE_CURRENT_LIST_DIR}/utils/timer.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils/parallel_foreach_element.h"
)

if(${CGOGN_EXTERNAL_TEMPLATES})
	target_sources(${PROJECT_NAME}
		PRIVATE
			"${CMAKE_CURRENT_LIST_DIR}/external_templates.cpp"
	)
endif()

# use of target_compile_options to have transitive flags
if(CGOGN_USE_SIMD)
	target_compile_options(${PROJECT_NAME} PUBLIC ${CGOGN_SSE_FLAGS})
	target_compile_definitions(${PROJECT_NAME} PUBLIC "CGOGN_USE_SIMD")
else()
	target_compile_definitions(${PROJECT_NAME} PUBLIC "EIGEN_DONT_VECTORIZE")
endif()

if(NOT MSVC)
	if (CGOGN_CPP_STD STREQUAL "11")
		target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
		target_compile_definitions(${PROJECT_NAME} PUBLIC "CGOGN_CPP_STD=11")
	elseif(CGOGN_CPP_STD STREQUAL "14")
		target_compile_definitions(${PROJECT_NAME} PUBLIC "CGOGN_CPP_STD=14")
		target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++14")
	else(CGOGN_CPP_STD STREQUAL "11")
		target_compile_definitions(${PROJECT_NAME} PUBLIC "CGOGN_CPP_STD=17")
		target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++17")
	endif(CGOGN_CPP_STD STREQUAL "11")
	if(${CGOGN_USE_CXX11_ABI})
		target_compile_definitions(${PROJECT_NAME} PUBLIC "_GLIBCXX_USE_CXX11_ABI")
		remove_flags(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI")
	endif()
	if (${CGOGN_USE_GLIBCXX_DEBUG})
		target_compile_definitions(${PROJECT_NAME} PUBLIC "_GLIBCXX_DEBUG")
		remove_flags(CMAKE_CXX_FLAGS "-D_GLIBCXX_DEBUG")
		if(${CGOGN_USE_GLIBCXX_DEBUG_PEDANTIC})
			target_compile_definitions(${PROJECT_NAME} PUBLIC "_GLIBCXX_DEBUG_PEDANTIC")
			remove_flags(CMAKE_CXX_FLAGS "-D_GLIBCXX_DEBUG_PEDANTIC")
		endif()
	endif()
	if(${CGOGN_USE_PARALLEL_GLIBCXX} AND (NOT ${CGOGN_USE_GLIBCXX_DEBUG}))
		target_compile_definitions(${PROJECT_NAME} PUBLIC "_GLIBCXX_PARALLEL")
		remove_flags(CMAKE_CXX_FLAGS "-D_GLIBCXX_PARALLEL")
	endif()
	target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-unknown-pragmas")
endif()

if(MSVC)
	target_compile_definitions(${PROJECT_NAME} PUBLIC "CGOGN_CPP_STD=11")
	target_compile_definitions(${PROJECT_NAME} PUBLIC "_USE_MATH_DEFINES")
	## CGOGN_WIN_VER : has value 61 for windows 7, 62 for windows 8, 63 for windows 8.1, 100 for windows 10
	set(WIN_VERSION "")
	string(REPLACE "." "" WIN_VERSION ${CMAKE_SYSTEM_VERSION})
	target_compile_definitions(${PROJECT_NAME} PUBLIC "CGOGN_WIN_VER=${WIN_VERSION}")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "CGOGN_CORE_DLL_EXPORT")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TERMCOLOR_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
	$<INSTALL_INTERFACE:include/cgogn/thirdparty>
)

if(${CGOGN_EXTERNAL_TEMPLATES})
	target_compile_definitions(${PROJECT_NAME} PUBLIC "CGOGN_USE_EXTERNAL_TEMPLATES")
endif()

target_compile_definitions(${PROJECT_NAME} PUBLIC "CGOGN_LITTLE_ENDIAN=1234")
target_compile_definitions(${PROJECT_NAME} PUBLIC "CGOGN_BIG_ENDIAN=4321")
if(CGOGN_TEST_BIG_ENDIAN)
	target_compile_definitions(${PROJECT_NAME} PUBLIC "CGOGN_ENDIANNESS=CGOGN_BIG_ENDIAN")
else()
	target_compile_definitions(${PROJECT_NAME} PUBLIC "CGOGN_ENDIANNESS=CGOGN_LITTLE_ENDIAN")
endif()


install(FILES "dll.h" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/core/)
install(DIRECTORY basic cmap container utils graph
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cgogn/core/
	FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CGOGN_SOURCE_DIR};${CGOGN_THIRDPARTY_TERMCOLOR_INCLUDE_DIR}" "include;include/cgogn/thirdparty")

# VisualStudio nice source organisation
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
