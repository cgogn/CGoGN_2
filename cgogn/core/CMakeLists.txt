project(cgogn_core
	LANGUAGES CXX
)



set(HEADER_BASIC
	basic/dart.h
	basic/dart_marker.h
	basic/cell.h
	basic/cell_marker.h
)

set(HEADER_CMAP
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
	cmap/cmap2_quad.h
	cmap/cmap3_tetra.h
	cmap/cmap3_hexa.h
)

set(HEADER_CONTAINER
	container/chunk_array_container.h
	container/chunk_array_factory.h
	container/chunk_array_gen.h
	container/chunk_array.h
	container/chunk_stack.h
)

set(HEADER_GRAPH
	graph/undirected_graph.h
	graph/undirected_graph_builder.h
)

set(HEADER_UTILS
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
	utils/timer.h
	utils/parallel_foreach_element.h
)

set(HEADER_FILES dll.h ${HEADER_BASIC} ${HEADER_CMAP} ${HEADER_CONTAINER} ${HEADER_GRAPH} ${HEADER_UTILS})


set(SOURCE_CMAP
	cmap/map_base_data.cpp
	cmap/cmap0.cpp
	cmap/cmap1.cpp
	cmap/cmap2.cpp
	cmap/cmap3.cpp
	cmap/cmap2_tri.cpp
	cmap/cmap2_quad.cpp
	cmap/cmap3_tetra.cpp
	cmap/cmap3_hexa.cpp
)
set(SOURCE_CONTAINER
	container/chunk_array_container.cpp
	container/chunk_array_gen.cpp
	container/chunk_array.cpp
	container/chunk_stack.cpp
	container/chunk_array_container.cpp
	container/chunk_array_factory.cpp
)
set(SOURCE_GRAPH
	graph/undirected_graph.cpp
)
set(SOURCE_UTILS
	utils/assert.cpp
	utils/name_types.cpp
	utils/thread.cpp
	utils/thread_pool.cpp
	utils/serialization.cpp
	utils/logger.cpp
	utils/log_entry.cpp
	utils/logger_output.cpp
	utils/log_stream.cpp
	utils/masks.cpp
	utils/string.cpp
	utils/timer.cpp
)

set(SOURCE_FILES ${SOURCE_CMAP} ${SOURCE_CONTAINER} ${SOURCE_GRAPH} ${SOURCE_UTILS})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})


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



install(FILES "dll.h" DESTINATION "include/cgogn/core/")
install(DIRECTORY basic cmap container utils graph
	DESTINATION "include/cgogn/core/"
	FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CGOGN_SOURCE_DIR};${CGOGN_THIRDPARTY_TERMCOLOR_INCLUDE_DIR}" "include;include/cgogn/thirdparty")

# VisualStudio nice source organisation
source_group("CMAP" FILES ${HEADER_CMAP} ${SOURCE_CMAP})
source_group("CONTAINER" FILES ${HEADER_CONTAINER} ${SOURCE_CONTAINER})
source_group("GRAPH" FILES ${HEADER_GRAPH} ${SOURCE_GRAPH})
source_group("UTILS" FILES ${HEADER_UTILS} ${SOURCE_UTILS})
source_group("BASIC" FILES ${HEADER_BASIC})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
