set(CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "TinyXml2 include directory")

cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(tinyxml2)

# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/.")

if(MSVC)
	add_definitions("-D_CRT_SECURE_NO_WARNINGS")
endif(MSVC)



add_library(${PROJECT_NAME} STATIC tinyxml2.cpp tinyxml2.h)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS "TINYXML2_EXPORT")

if(NOT MSVC)
	target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

install(FILES "tinyxml2.h" DESTINATION "include/thirdparty/TinyXml2")

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "include")
