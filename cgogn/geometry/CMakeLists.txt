project(cgogn_geometry
	LANGUAGES CXX
)

add_library(${PROJECT_NAME} SHARED "")

target_sources(${PROJECT_NAME}
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/types/plane_3d.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/types/quadric.cpp"

        "${CMAKE_CURRENT_LIST_DIR}/algos/bounding_box.h"
        "${CMAKE_CURRENT_LIST_DIR}/algos/feature.h"
        "${CMAKE_CURRENT_LIST_DIR}/algos/area.h"
        "${CMAKE_CURRENT_LIST_DIR}/algos/centroid.h"
        "${CMAKE_CURRENT_LIST_DIR}/algos/curvature.h"
        "${CMAKE_CURRENT_LIST_DIR}/algos/normal.h"
        "${CMAKE_CURRENT_LIST_DIR}/algos/ear_triangulation.h"
        "${CMAKE_CURRENT_LIST_DIR}/algos/picking.h"
        "${CMAKE_CURRENT_LIST_DIR}/algos/selection.h"
        "${CMAKE_CURRENT_LIST_DIR}/algos/filtering.h"
        "${CMAKE_CURRENT_LIST_DIR}/algos/length.h"
        "${CMAKE_CURRENT_LIST_DIR}/algos/angle.h"

        "${CMAKE_CURRENT_LIST_DIR}/functions/basics.h"
        "${CMAKE_CURRENT_LIST_DIR}/functions/area.h"
        "${CMAKE_CURRENT_LIST_DIR}/functions/normal.h"
        "${CMAKE_CURRENT_LIST_DIR}/functions/orientation.h"
        "${CMAKE_CURRENT_LIST_DIR}/functions/inclusion.h"
        "${CMAKE_CURRENT_LIST_DIR}/functions/intersection.h"
        "${CMAKE_CURRENT_LIST_DIR}/functions/distance.h"

        "${CMAKE_CURRENT_LIST_DIR}/types/aabb.h"
        "${CMAKE_CURRENT_LIST_DIR}/types/obb.h"
        "${CMAKE_CURRENT_LIST_DIR}/types/eigen.h"
        "${CMAKE_CURRENT_LIST_DIR}/types/geometry_traits.h"
        "${CMAKE_CURRENT_LIST_DIR}/types/plane_3d.h"
        "${CMAKE_CURRENT_LIST_DIR}/types/quadric.h"
        "${CMAKE_CURRENT_LIST_DIR}/types/vec.h"
    )

if(${CGOGN_EXTERNAL_TEMPLATES})
    target_sources(${PROJECT_NAME}
        PRIVATE
            "${CMAKE_CURRENT_LIST_DIR}/external_templates.cpp"
    )
endif()


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
#source_group("ALGOS" FILES ${HEADER_ALGOS} ${SOURCE_ALGOS})
#source_group("TYPES" FILES ${HEADER_TYPES} ${SOURCE_TYPES})
#source_group("FUNCTIONS" FILES ${HEADER_FUNCTIONS})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER cgogn)
