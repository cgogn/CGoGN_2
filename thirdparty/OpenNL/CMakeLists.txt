#version OpenNL PSM_1.3.10 (2016-10-18 14:51)
 
set(CGOGN_THIRDPARTY_OPEN_NL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "OpenNL include directory")

project(OpenNL
	LANGUAGES C
)

cgogn_reset_warning_level()
set(HEADER_FILES
	OpenNL_psm.h
)

set(SOURCE_FILES
	OpenNL_psm.c
)


add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

install(FILES "OpenNL_psm.h" DESTINATION "include/thirdparty/OpenNL")

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "include")
