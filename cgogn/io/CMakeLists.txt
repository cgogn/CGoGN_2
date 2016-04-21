project(cgogn_io
	LANGUAGES CXX
)

find_package(ZLIB)
find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(ply REQUIRED)
find_package(lm6 REQUIRED)
find_package(tinyxml2 REQUIRED)

set(HEADER_FILES
	surface_import.h
	volume_import.h
	map_import.h
	map_export.h
	import_ply_data.h
	io_utils.h
	data_io.h
	dll.h
	c_locale.h
	vtk_io.h
	off_io.h
	obj_io.h
	ply_io.h
	lm6_io.h
	mesh_io_gen.h
	msh_io.h
	tetgen_io.h
	nastran_io.h
	tet_io.h
)

set(SOURCE_FILES
	surface_import.cpp
	volume_import.cpp
	import_ply_data.cpp
	mesh_io_gen.cpp
	io_utils.cpp
	vtk_io.cpp
	lm6_io.cpp
	data_io.cpp
	msh_io.cpp
	obj_io.cpp
	off_io.cpp
	ply_io.cpp
	tetgen_io.cpp
	nastran_io.cpp
	tet_io.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_LM6_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_PLY_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${ZLIB_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>
)

if (${ZLIB_FOUND})
	target_compile_definitions(${PROJECT_NAME} PUBLIC "-DZLIB_CONST")
	target_compile_definitions(${PROJECT_NAME} PUBLIC "-DCGOGN_WITH_ZLIB")
endif()

target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_geometry_LIBRARIES} ${ZLIB_LIBRARIES} ${ply_LIBRARIES} ${lm6_LIBRARIES} ${tinyxml2_LIBRARIES})

file(GLOB HEADERS "." "*.h")
install(FILES ${HEADERS}
	DESTINATION include/cgogn/io)

######## 1. Build tree

export(TARGETS ${PROJECT_NAME}
	FILE "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Targets.cmake")

set(CGOGN_IO_INCLUDE_DIRS "${CGOGN_SOURCE_DIR}")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake"
	PATH_VARS CGOGN_IO_INCLUDE_DIRS
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
set(CGOGN_IO_INCLUDE_DIRS "include")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake"
	PATH_VARS CGOGN_IO_INCLUDE_DIRS
	INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
)

install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}")
install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}" RENAME "${PROJECT_NAME}Config.cmake")


add_subdirectory(mesh_generation)
add_subdirectory(examples)
