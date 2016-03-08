project(cgogn_tiling
	LANGUAGES CXX
)

set(HEADER_FILES
	dll.h
	tiling.h
	triangular.h
)

set(SOURCE_FILES
	triangular.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include/cgogn/tiling>
)

target_link_libraries(${PROJECT_NAME} cgogn_core cgogn_geometry)

install(DIRECTORY .
	DESTINATION include/cgogn/tiling
	FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

add_subdirectory(examples)

if(CGOGN_BUILD_TESTS)
	#add_subdirectory(tests)
endif()
