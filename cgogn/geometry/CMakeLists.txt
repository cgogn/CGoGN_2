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
	algos/curvature.h
	algos/normal.h
	algos/ear_triangulation.h
	algos/picking.h
	algos/selection.h
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
	types/aabb.h
	types/obb.h
	types/eigen.h
	types/geometry_traits.h
	types/plane_3d.h
	types/vec.h
)

set(SOURCE_FILES
	algos/angle.cpp
	algos/selection.cpp
	types/aabb.cpp
	types/obb.cpp
	types/plane_3d.cpp
	types/vec.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_GEOMETRY_DLL_EXPORT")
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

cgogn_create_package("${CGOGN_SOURCE_DIR};${CGOGN_THIRDPARTY_EIGEN3_INCLUDE_DIR}" "include")
