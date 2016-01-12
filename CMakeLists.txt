cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(CGOGN_VERSION_MAJOR 2)
set(CGOGN_VERSION_MINOR 0)
set(CGOGN_VERSION_PATCH 0)

project(CGoGN
	VERSION ${CGOGN_VERSION_MAJOR}.${CGOGN_VERSION_MINOR}.${CGOGN_VERSION_PATCH}
	LANGUAGES CXX
)

#### Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()


#### Here are located the FindPackages that we need
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

#### Compile Options
include(cmake/CompilerOptions.cmake)
include(cmake/GenerateCppcheck.cmake)
include(cmake/EnableCoverageReport.cmake)

#### Build configuration
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CGOGN_SOURCE_DIR ${CMAKE_SOURCE_DIR}/modules)


#### ThirdParty options
option(CGOGN_PROVIDE_EIGEN "Use the version of eigen that is packaged with CGoGN" ON)
option(CGOGN_BUILD_TESTS "build cgogn unit tests using google test framework" ON)


#### Continuous integration options
option(CGOGN_WITH_GPROF "Builds the project for performance analysis with gprof" OFF)
option(CGOGN_WITH_GCOV "Builds the project for coverage analysis with gcov" OFF)
option(CGOGN_WITH_PPROF "Profile the project using gprof" OFF)
option(CGOGN_WITH_ASAN "Builds the project with Google's AddressSanitizer" OFF)
option(CGOGN_WITH_TSAN "Builds the project with Google's ThreadSanitizer" OFF)
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "AppleClang" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    option(CGOGN_WITH_CLANGSA "Builds the project for performance analysis with clang static analyser" OFF)
endif()


#### RPATH
if(UNIX)
    # RPATH is a field in ELF binaries that is used as a hint by the system
    # loader to find needed shared libraries.
    #
    # In the build directory, cmake creates binaries with absolute paths in
    # RPATH.  And by default, it strips RPATH from installed binaries.  Here we
    # use CMAKE_INSTALL_RPATH to set a relative RPATH.  By doing so, we avoid
    # the need to play with LD_LIBRARY_PATH to get applications to run.
    set(CMAKE_INSTALL_RPATH "../lib:../../lib")

    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(CMAKE_MACOSX_RPATH ON)
    endif()
endif(UNIX)

## Deduce build type of not forced by setting the CMAKE_BUILD_TYPE var
deduce_build_type()

if(CGOGN_BUILD_TESTS)
    if(NOT gtest_inited)
        set(gtest_inited ON CACHE INTERNAL "" FORCE)
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    endif()
endif(CGOGN_BUILD_TESTS)

add_subdirectory(thirdparty)
add_subdirectory(${CGOGN_SOURCE_DIR})



##  TODO a mettre dans un fichier cmake particulier

# --- coverage ---
ENABLE_COVERAGE_REPORT(TARGETS ${CGOGN_SOURCE_DIR})

# --- CPPCheck ---
GENERATE_CPPCHECK( SOURCES ${CGOGN_SOURCE_DIR} 
                   ENABLE_IDS all 
                   INCLUDES ${CGOGN_SOURCE_DIR}
                   PLATFORM_FLAGS "-UNDEBUG -UWIN32 -U__func__ -U__FUNCTION__ -U__GNUG__ -U__clang__ -U_MSC_VER"
                   SYSTEM_INCLUDE_WARNING_SUPPRESSION 
                   INLINE_SUPPRESSION
                   HTML_REPORT)
