project(cgogn_multiresolution
	LANGUAGES CXX
)

set(HEADER_FILES
	dll.h

	cph/attribute_handler_cph.h
	cph/cph_base.h
	cph/cph2.h
	cph/cph3.h
	cph/ihcmap2.h
	cph/ihcmap2_adaptive.h
	cph/ihcmap2_regular.h
	cph/ihcmap3.h

	mrcmap/mr_base.h
	mrcmap/mrcmap2.h

	mra/mr_analysis.h
	mra/lerp_triquad_mra.h
)

set(SOURCE_FILES
	cph/cph_base.cpp
	cph/cph2.cpp
	cph/cph3.cpp

	cph/ihcmap2.cpp
	cph/ihcmap2_adaptive.cpp
	cph/ihcmap2_regular.cpp
	cph/ihcmap3.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include/cgogn/multiresolution>
)

target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_geometry)

install(DIRECTORY .
	DESTINATION include/cgogn/multiresolution
	FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

add_subdirectory(examples)
