
include(CMakeParseArguments)


function(add_flags _var)
	string(REPLACE " " ";" string "${${_var}};${ARGN}")
	list(REMOVE_DUPLICATES string)
	string(REPLACE ";" " " string "${string}")
	set(${_var} ${string} PARENT_SCOPE)
endfunction()

function(remove_flags _var)
    string(REPLACE " " ";" flags "${${_var}}")
    list(REMOVE_ITEM flags ${ARGN})
    string(REPLACE ";" " " flags "${flags}")
    set(${_var} ${flags} PARENT_SCOPE)
endfunction()

#!
# @brief Add sources from directories
# @details
# Add the sources from the specified \p directories to variable \p var
# and place them in the specified \p folder if non empty.
# @param[out] var name of the variable that receives the result list
# @param[in] folder the name of the folder
# @param[in] directories list of directories to scan
#
function(aux_source_directories var folder)
	set(sources)
	foreach(dir ${ARGN})
		file(GLOB _sources RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${dir}/*.[ch]" "${dir}/*.[ch]pp")
		list(APPEND sources ${_sources})
	endforeach()

	if( NOT folder STREQUAL "")
		source_group(${folder} FILES ${sources})
	endif()

	#message("\nDEBUG: aux_source_directories: current_source_dir=${CMAKE_CURRENT_SOURCE_DIR} dirs=${ARGN}):\n${sources}\n")
	set(${var} ${${var}} ${sources} PARENT_SCOPE)
endfunction()

#!
# @brief automatic deduction of CMAKE_BUILD_TYPE from CMAKE_CURRENT_BINARY_DIR
# @details
# if CMAKE_CURRENT_BINARY_DIR end with Debug/debug/DEBUG, set Debug mode else Release mode.
#
function(deduce_build_type)
set(BIN_DIR "")
string(TOLOWER ${CMAKE_CURRENT_BINARY_DIR} BIN_DIR)
if (${BIN_DIR} MATCHES "(.*)debug")
	set(CMAKE_BUILD_TYPE "Debug" PARENT_SCOPE)
else()
	if (${BIN_DIR} MATCHES "(.*)release")
		set(CMAKE_BUILD_TYPE "Release" PARENT_SCOPE)
	endif()
endif()
endfunction(deduce_build_type)

##############################################################################################
#                                 cgogn_create_package macro                                 #
# This macro is a helper to create package configuration and version files. These files are  #
# needed when using the find_package command.                                                #
# This macro generate 2 versions of each file : one for the build tree and another for the   #
# install tree.                                                                              #
# Build tree:                                                                                #
# 1.<build-dir>/lib/cmake/<cmake-project-name>/<cmake-project-name>Targets.cmake             #
# 2.<build-dir>/lib/cmake/<cmake-project-name>/<cmake-project-name>Config.cmake              #
# 3.<build-dir>/lib/cmake/<cmake-project-name>/<cmake-project-name>ConfigVersion.cmake       #
#                                                                                            #
# Install tree:                                                                              #
# 1.<install-dir>/lib/cmake/<cmake-project-name>/<cmake-project-name>Targets.cmake           #
# 2.<install-dir>/lib/cmake/<cmake-project-name>/<cmake-project-name>Config.cmake            #
# 3.<install-dir>/lib/cmake/<cmake-project-name>/<cmake-project-name>ConfigVersion.cmake     #
#                                                                                            #
# Usage example : find_package(cgogn_core); find_package(cgogn_io)                           #
# Note: template config files are located in  the cmake/ConfigFiles directory.               #
# By convention they have to define the following two variables:                             #
# cmake/<cmake-project-name>_LIBRARIES                                                       #
# cmake/<cmake-project-name>_INCLUDE_DIRS                                                    #
##############################################################################################

macro(cgogn_create_package package_root_dir include_dirs_build_tree include_dirs_install_tree)

######## 1. Build tree

set(UPPER_NAME "")
string(TOUPPER ${PROJECT_NAME} UPPER_NAME)
set(${UPPER_NAME}_INCLUDE_DIRS ${include_dirs_build_tree})

export(TARGETS ${PROJECT_NAME}
	FILE "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Targets.cmake"
)

#message(${package_root_dir})

configure_package_config_file(
#	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${package_root_dir}/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake"
	PATH_VARS ${UPPER_NAME}_INCLUDE_DIRS 
	INSTALL_DESTINATION "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}"
)

write_basic_package_version_file(
	"${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake"
	VERSION ${CGOGN_VERSION_MAJOR}.${CGOGN_VERSION_MINOR}.${CGOGN_VERSION_PATCH}
	COMPATIBILITY ExactVersion
)

######## 2. Install tree

set(${UPPER_NAME}_INCLUDE_DIRS ${include_dirs_install_tree})

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)
install(EXPORT ${PROJECT_NAME}Targets DESTINATION "lib/cmake/${PROJECT_NAME}")

## <package_name>ConfigVersion.cmake
write_basic_package_version_file(
	"${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake"
	VERSION ${CGOGN_VERSION_MAJOR}.${CGOGN_VERSION_MINOR}.${CGOGN_VERSION_PATCH}
	COMPATIBILITY ExactVersion
)

## <package_name>Config.cmake
set(CURRENT_LIBRARY "${PROJECT_NAME}")
configure_package_config_file(
#	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${package_root_dir}/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake"
	PATH_VARS ${UPPER_NAME}_INCLUDE_DIRS
	INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
)

install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}")
install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}" RENAME "${PROJECT_NAME}Config.cmake")

endmacro()


##############################################################################################

macro(subdirlist result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
		list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()
