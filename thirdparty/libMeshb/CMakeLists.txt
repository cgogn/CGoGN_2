set(CGOGN_THIRDPARTY_MESHB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "MESHB include directory")

project(Meshb
	LANGUAGES C
	)

cgogn_reset_warning_level()

set(HEADER_FILES
	libmeshb.h
	)

set(SOURCE_FILES
	libmeshb.c
	)




add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SOURCE_FILES})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/tools)
add_executable(transmesh transmesh.c)
target_include_directories(transmesh PRIVATE
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_MESHB_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
)
target_link_libraries(transmesh ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

install(FILES "libmeshb.h" DESTINATION "include/cgogn/thirdparty")

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}" "include/cgogn/thirdparty")
