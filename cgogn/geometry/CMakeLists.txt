project(cgogn_geometry
	LANGUAGES CXX
)

set(HEADER_FILES
	dll.h
	algos/bounding_box.h
	algos/area.h
	algos/centroid.h
	algos/normal.h
	algos/ear_triangulation.h
	algos/picking.h
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

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_EIGEN3_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include/cgogn/geometry>
)

target_link_libraries(${PROJECT_NAME} cgogn_core)

install(DIRECTORY .
	DESTINATION include/cgogn/geometry
	FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

if(CGOGN_BUILD_TESTS)
	add_subdirectory(tests)
endif()
