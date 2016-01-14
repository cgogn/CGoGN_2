project(cgogn_io
	LANGUAGES CXX
)

set(HEADER_FILES
	surface_import.h
	map_import.h
	dll.h
)

set(SOURCE_FILES
	surface_import.cpp
)

 add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})


# target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")

# set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

# target_include_directories(${PROJECT_NAME} PUBLIC
# 	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
# 	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}/src>
# 	$<INSTALL_INTERFACE:include/cgogn/cgogn_io>
# )


 # set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

 # target_include_directories(${PROJECT_NAME} PUBLIC
	# $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	# $<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	# $<INSTALL_INTERFACE:include/cgogn/io>
 # )

 # target_link_libraries(${PROJECT_NAME} cgogn_utils cgogn_core)


# target_link_libraries(${PROJECT_NAME} cgogn_core)

# install(DIRECTORY basic container map
# 	DESTINATION include/cgogn/cgogn_core
# 	FILES_MATCHING PATTERN "*.h"
# )

 install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
 )
