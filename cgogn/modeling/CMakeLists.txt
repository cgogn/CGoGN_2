project(cgogn_modeling
	LANGUAGES CXX
)

set(HEADER_FILES
	dll.h
	algos/pliant_remeshing.h
)

set(SOURCE_FILES
	algos/pliant_remeshing.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_EIGEN3_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include/cgogn/modeling>
)

target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_geometry)

install(DIRECTORY .
	DESTINATION include/cgogn/modeling
	FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

add_subdirectory(examples)

#if(CGOGN_BUILD_TESTS)
#	add_subdirectory(tests)
#endif()


