set(TinyXml2_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "TinyXml2 include directory")

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
#install(TARGETS tinyxml2
#        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
#        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

#install(FILES tinyxml2.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

#foreach(p LIB INCLUDE)
#	set(var CMAKE_INSTALL_${p}DIR)
#	if(NOT IS_ABSOLUTE "${${var}}")
#		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
#	endif()
#endforeach()

#configure_file(tinyxml2.pc.in tinyxml2.pc @ONLY)
#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/tinyxml2.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

#add_test(xmltest ${SAMPLE_NAME} COMMAND $<TARGET_FILE:${SAMPLE_NAME}>)
