project(cgogn_geometry
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)

set(HEADER_FILES
	dll.h
	algos/bounding_box.h
	algos/feature.h
	algos/area.h
	algos/centroid.h
	algos/normal.h
	algos/ear_triangulation.h
	algos/picking.h
	algos/filtering.h
	algos/length.h
	algos/angle.h
	functions/basics.h
	functions/area.h
	functions/normal.h
	functions/orientation.h
	functions/inclusion.h
	functions/intersection.h
	functions/distance.h
	types/bounding_box.h
	types/eigen.h
	types/geometry_traits.h
	types/plane_3d.h
	types/vec.h
)

set(SOURCE_FILES
	types/plane_3d.cpp
	types/vec.cpp
	types/bounding_box.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_EIGEN3_INCLUDE_DIR}>
)

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES})

install(FILES "dll.h" DESTINATION "include/cgogn/geometry")
install(DIRECTORY algos functions types
	DESTINATION "include/cgogn/geometry"
	FILES_MATCHING PATTERN "*.h"
)

######## 1. Build tree

export(TARGETS ${PROJECT_NAME}
	FILE "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Targets.cmake")

set(CGOGN_GEOMETRY_INCLUDE_DIRS "${CGOGN_SOURCE_DIR};${CGOGN_THIRDPARTY_EIGEN3_INCLUDE_DIR}")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake"
	PATH_VARS CGOGN_GEOMETRY_INCLUDE_DIRS
	INSTALL_DESTINATION "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}"
)

## <package_name>ConfigVersion.cmake
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
set(CGOGN_GEOMETRY_INCLUDE_DIRS "include")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake"
	PATH_VARS CGOGN_GEOMETRY_INCLUDE_DIRS
	INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
)

install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}")
install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}" RENAME "${PROJECT_NAME}Config.cmake")

add_subdirectory(examples)

if(CGOGN_BUILD_TESTS)
	add_subdirectory(tests)
endif()
