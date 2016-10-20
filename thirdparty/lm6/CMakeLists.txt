set(CGOGN_THIRDPARTY_LM6_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "LM6 include directory")

project(lm6
	LANGUAGES C
	)

cgogn_reset_warning_level()

set(HEADER_FILES
	libmesh6.h
	)

set(SOURCE_FILES
	libmesh6.c
	)




add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SOURCE_FILES})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/tools)
add_executable(transmesh transmesh.c)
target_include_directories(transmesh PRIVATE
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_LM6_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
)
target_link_libraries(transmesh ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

install(FILES "libmesh6.h" DESTINATION "include/thirdparty/lm6")

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "include")
