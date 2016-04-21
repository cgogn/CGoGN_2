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
	cmap/attribute_handler.h

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

# use of target_compile_options to have a transitive c++11 flag
if(NOT MSVC)
	target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
endif()
if(MSVC)
	target_compile_options(${PROJECT_NAME} PUBLIC "-D_USE_MATH_DEFINES")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TERMCOLOR_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:"include">
)

install(FILES "dll.h" DESTINATION "include/cgogn/core/")
install(DIRECTORY basic cmap container utils
	DESTINATION "include/cgogn/core/"
	FILES_MATCHING PATTERN "*.h"
)



######## 1. Build tree

export(TARGETS ${PROJECT_NAME}
FILE "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Targets.cmake")

set(CGOGN_CORE_INCLUDE_DIRS "${CGOGN_SOURCE_DIR};${CGOGN_THIRDPARTY_TERMCOLOR_INCLUDE_DIR}")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake"
	PATH_VARS CGOGN_CORE_INCLUDE_DIRS
	INSTALL_DESTINATION "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}"
)

write_basic_package_version_file(
	"${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake"
	VERSION ${CGOGN_VERSION_MAJOR}.${CGOGN_VERSION_MINOR}.${CGOGN_VERSION_PATCH}
	COMPATIBILITY ExactVersion
)

######## 2. Install tree

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION "bin"
	LIBRARY DESTINATION "lib"
	ARCHIVE DESTINATION "lib"
)
install(EXPORT ${PROJECT_NAME}Targets DESTINATION "lib/cmake/${PROJECT_NAME}")

## <package_name>ConfigVersion.cmake
write_basic_package_version_file(
	"${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake"
	VERSION ${CGOGN_VERSION_MAJOR}.${CGOGN_VERSION_MINOR}.${CGOGN_VERSION_PATCH}
	COMPATIBILITY ExactVersion
)

## <package_name>Config.cmake
set(CURRENT_LIBRARY "${PROJECT_NAME}")
set(CGOGN_CORE_INCLUDE_DIRS "include")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake"
	PATH_VARS CGOGN_CORE_INCLUDE_DIRS
	INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
)

install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}")
install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}" RENAME "${PROJECT_NAME}Config.cmake")

add_subdirectory(examples)

if(CGOGN_BUILD_TESTS)
	add_subdirectory(tests)
endif()
