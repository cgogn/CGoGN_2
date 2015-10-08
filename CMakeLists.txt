##########################################
## CGoGN root CMakeList
##########################################

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "build/")

cmake_minimum_required(VERSION 2.8.11)



#### Checking the current OS



#CMAKE_SYSTEM_NAME

# Linux

 #Windows

 #Darwin

#file(MAKE_DIRECTORY ${directory})


MESSAGE(STATUS "-------------- Checking for Cmake --------------")



foreach(config Release Debug)
	#CONACT(PLATFORM  ${config})
	MESSAGE(STATUS "-------------- Checking makefiles for ${config} mode --------------")

	

endforeach(config)


#SET(CMAKE_BUILD_TYPE )

#SET(CGoGN_PLATFORM )



# CMAKE_SIZEOF_VOID_P

##### Basic information

# Determine the current build-os (build-platform without the compiler info)
#string(REGEX REPLACE "-[^-]+$" "" CGOGN_OS ${CGoGN_PLATFORM})

# Determine the current build date
#string(TIMESTAMP CGOGN_BUILD_DATE "%Y-%m-%d %H:%M:%S")
#string(TIMESTAMP YEAR "%Y")
# TODO: Determine the current build number using jenkins


##### GIT tag info
# http://stackoverflow.com/a/4120179
# TODO
# decider d'utiliser les tag de git
# extraire les informations de la commande git describe --always


##### Build configuration

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

#include(${CMAKE_SOURCE_DIR}/cmake/utilities.cmake)
#include(${CMAKE_SOURCE_DIR}/cmake/platforms/${CGoGN_PLATFORM}/config.cmake)

##### CGOGN sources

#include_directories(${CMAKE_SOURCE_DIR}/src/lib)

#add_subdirectory(src/lib/core)

#add_subdirectory(src/bin)
#add_subdirectory(src/test)