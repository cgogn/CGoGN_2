project(cgogn_geometry
	LANGUAGES CXX
)

set(HEADER_ALGOS
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
)
set(HEADER_FUNCTIONS
	functions/basics.h
	functions/area.h
	functions/normal.h
	functions/orientation.h
	functions/inclusion.h
	functions/intersection.h
	functions/distance.h
)
set(HEADER_TYPES
	types/aabb.h
	types/obb.h
	types/eigen.h
	types/geometry_traits.h
	types/plane_3d.h
	types/quadric.h
	types/vec.h
)
set(HEADER_FILES dll.h ${HEADER_ALGOS} ${HEADER_FUNCTIONS} ${HEADER_TYPES})


set(SOURCE_ALGOS
	algos/angle.cpp
	algos/selection.cpp
)
set(SOURCE_TYPES
	types/aabb.cpp
	types/obb.cpp
	types/plane_3d.cpp
	types/quadric.cpp
	types/vec.cpp
)
set(SOURCE_FILES ${SOURCE_ALGOS} ${SOURCE_TYPES})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCGOGN_GEOMETRY_DLL_EXPORT")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include/cgogn/thirdparty>
#	$<INSTALL_INTERFACE:include/cgogn/thirdparty/eigen/unsupported>
)

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} cgogn_core)

install(FILES "dll.h" DESTINATION "include/cgogn/geometry")
install(DIRECTORY algos functions types
	DESTINATION "include/cgogn/geometry"
	FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CGOGN_SOURCE_DIR};${CGOGN_THIRDPARTY_EIGEN3_INCLUDE_DIR}" "include;include/cgogn/thirdparty")

# VisualStudio nice source organisation
source_group("ALGOS" FILES ${HEADER_ALGOS} ${SOURCE_ALGOS})
source_group("TYPES" FILES ${HEADER_TYPES} ${SOURCE_TYPES})
source_group("FUNCTIONS" FILES ${HEADER_FUNCTIONS})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
