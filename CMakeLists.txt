##########################################
## CGoGN root CMakeList
##########################################

cmake_minimum_required(VERSION 2.8.11)


##### Basic information

# Determine the current build-os (build-platform without the compiler info)
string(REGEX REPLACE "-[^-]+$" "" CGOGN_OS ${CGoGN_PLATFORM})

# Determine the current build date
string(TIMESTAMP CGOGN_BUILD_DATE "%Y-%m-%d %H:%M:%S")
string(TIMESTAMP YEAR "%Y")

# TODO: Determine the current build number using jenkins


##### GIT tag info
# http://stackoverflow.com/a/4120179
# TODO
# decider d'utiliser les tag de git
# extraire les informations de la commande git describe --always


##### Build configuration
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

include(${CMAKE_SOURCE_DIR}/cmake/utilities.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/platforms/${CGoGN_PLATFORM}/config.cmake)


##### CGOGN sources
include_directories(${CMAKE_SOURCE_DIR}/cgogn)

# add each subdirectory with a CMakeList
add_subdirectory(cgogn/core)
add_subdirectory(test)