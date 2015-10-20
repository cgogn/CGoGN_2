include(cmake/utilities.cmake)

if (UNIX AND NOT APPLE)
	if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
		include(cmake/platforms/Linux-gcc.cmake)
	endif()

	if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
		include(cmake/platforms/Linux.cmake)
	endif()
endif()

if(APPLE)
	if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
		include(cmake/platforms/Darwin.cmake)
	endif()

	if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
		include(cmake/platforms/Darwing-clang.cmake)
	endif()
endif()

if(WIN32)
	#TODO
endif()

