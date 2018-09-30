#version OpenNL PSM_1.3.10 (2016-10-18 14:51)
 
project(OpenNL
	LANGUAGES C
)

set(HEADER_FILES
	OpenNL_psm.h
)

set(SOURCE_FILES
	OpenNL_psm.c
)


add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/cgogn/thirdparty>
)

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_compile_definitions(${PROJECT_NAME} PUBLIC "_CRT_SECURE_NO_WARNINGS")

install(FILES "OpenNL_psm.h" DESTINATION "include/cgogn/thirdparty")

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}")

set_target_properties(OpenNL PROPERTIES FOLDER thirdparty)
