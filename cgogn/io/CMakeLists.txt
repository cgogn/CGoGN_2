project(cgogn_io
	LANGUAGES CXX
)

set(HEADER_FILES
	surface_import.h
	map_import.h
)

set(SOURCE_FILES
)

# add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

# set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

# target_include_directories(${PROJECT_NAME} PUBLIC
# 	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
# 	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
# 	$<INSTALL_INTERFACE:include/cgogn/io>
# )

# target_link_libraries(${PROJECT_NAME} cgogn_utils)

install(
	DIRECTORY .
	DESTINATION include/cgogn/io
	FILES_MATCHING PATTERN "*.h"
)

# install(TARGETS ${PROJECT_NAME}
# 	EXPORT ${PROJECT_NAME}Targets
# 	RUNTIME DESTINATION bin
# 	LIBRARY DESTINATION lib
# 	ARCHIVE DESTINATION lib
# )
