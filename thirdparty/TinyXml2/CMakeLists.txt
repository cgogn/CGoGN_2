cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(tinyxml2)

# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/.")

if(MSVC)
	add_definitions("-D_CRT_SECURE_NO_WARNINGS")
endif(MSVC)

add_library(${PROJECT_NAME} STATIC tinyxml2.cpp tinyxml2.h)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/cgogn/thirdparty>
)

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS "TINYXML2_EXPORT")

if(NOT MSVC)
	if (CGOGN_CPP_STD STREQUAL "11")
		target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
	elseif(CGOGN_CPP_STD STREQUAL "14")
		target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++14")
	else(CGOGN_CPP_STD STREQUAL "11")
		target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++17")
	endif(CGOGN_CPP_STD STREQUAL "11")
endif(NOT MSVC)
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

install(FILES "tinyxml2.h" DESTINATION "include/cgogn/thirdparty")

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}")
