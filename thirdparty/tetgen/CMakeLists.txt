set(CGOGN_THIRDPARTY_TETGEN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Tetgen include directory.")
# Set  the minimum  required version  of cmake  for a  project.
cmake_minimum_required(VERSION 3.0)

if(MSVC)
	add_definitions("-D_CRT_SECURE_NO_WARNINGS /W0")
	add_library(tet STATIC tetgen.h tetgen.cxx predicates.cxx)
else(MSVC)
	add_library(tet SHARED tetgen.h tetgen.cxx predicates.cxx)
endif(MSVC)

install(FILES "tetgen.h" DESTINATION "include/thirdparty/tetgen")

# Add an executable to the project using the specified source files.
# add_executable(tetgen tetgen.cxx predicates.cxx)

#Add a library to the project using the specified source files.
# In Linux/Unix, it will creates the libtet.a


#Set properties on a target.
#We use this here to set -DTETLIBRARY for when compiling the
#library
#set_target_properties(tet PROPERTIES "COMPILE_DEFINITIONS" TETLIBRARY)
