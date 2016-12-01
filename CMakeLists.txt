cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(CGOGN_VERSION_MAJOR 2)
set(CGOGN_VERSION_MINOR 0)
set(CGOGN_VERSION_PATCH 0)

project(CGoGN
	VERSION ${CGOGN_VERSION_MAJOR}.${CGOGN_VERSION_MINOR}.${CGOGN_VERSION_PATCH}
	LANGUAGES CXX C
)

enable_testing()

#### Default build type
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()

#### CGoGN PATH
set(CGOGN_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

#### Here are located the FindPackages that we need
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}")

#### Compile Options
include(cmake/CompilerOptions.cmake)
include(cmake/GenerateCppcheck.cmake)
include(cmake/EnableCoverageReport.cmake)

#### Build configuration
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CGOGN_SOURCE_DIR ${CGOGN_PATH})

### External Templates
option(CGOGN_EXTERNAL_TEMPLATES "Use external templates to reduce compile time" OFF)

### C++ 11/14/17
set(CGOGN_CPP_STD "11" CACHE STRING "The version of the c++ standard to use.")
if (NOT MSVC)
	set_property(CACHE CGOGN_CPP_STD PROPERTY STRINGS "11" "14" "17") # drop down list with cmake-gui
endif(NOT MSVC)

#### ThirdParty options
set(CGOGN_THIRDPARTY_DIR ${CGOGN_PATH}/thirdparty)
option(CGOGN_PROVIDE_EIGEN "Use the version of eigen that is packaged with CGoGN." ON)
option(CGOGN_PROVIDE_TINYXML2 "Use the version of tinyxml2 that is packaged with CGoGN." ON)
option(CGOGN_BUILD_TESTS "Build cgogn unit tests using google test framework." ON)
option(CGOGN_BUILD_EXAMPLES "Build some example apps." ON)
option(CGOGN_BUILD_BENCHS "Build the benchmarks using google benchmark framework." ON)
option(CGOGN_USE_OPENMP "Activate openMP directives." OFF)
option(CGOGN_ENABLE_LTO "Enable link-time optimizations (only with gcc)" ON)
option(CGOGN_INSANE_WARN_LEVEL "Set very very high warning compilation level." OFF)
if (NOT MSVC)
	option(CGOGN_USE_GLIBCXX_DEBUG "Use the debug version of STL (useful for bounds checking)." OFF)
	option(CGOGN_USE_GLIBCXX_DEBUG_PEDANTIC "Use an extremely picky debug version of STL." OFF)
	option(CGOGN_USE_PARALLEL_GLIBCXX "Highly experimental : compiles using the parallel mode." OFF)
	if (${CGOGN_USE_GLIBCXX_DEBUG_PEDANTIC})
	set(CGOGN_USE_GLIBCXX_DEBUG "ON")
	endif(${CGOGN_USE_GLIBCXX_DEBUG_PEDANTIC})
	option(CGOGN_USE_CXX11_ABI "use the CXX11 ABI." ON)
endif(NOT MSVC)
if (${CGOGN_USE_PARALLEL_GLIBCXX})
	set(CGOGN_USE_OPENMP "ON")
endif()
option(CGOGN_USE_QT "use Qt (5.4 min) for interface & rendering" ON)

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

if(${CGOGN_USE_OPENMP})
	find_package(OpenMP)
	if(${OPENMP_FOUND})
	add_flags(CMAKE_C_FLAGS ${OpenMP_C_FLAGS})
	add_flags(CMAKE_CXX_FLAGS  ${OpenMP_CXX_FLAGS})
	else()
	if ((${CMAKE_CXX_COMPILER_ID} MATCHES "Clang") AND (NOT (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.7")))
		add_flags(CMAKE_C_FLAGS "-fopenmp")
		add_flags(CMAKE_CXX_FLAGS  "-fopenmp")
	endif()
	endif()
endif()

if(CGOGN_BUILD_TESTS)
	if(NOT gtest_inited)
	set(gtest_inited ON CACHE INTERNAL "" FORCE)
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
	endif()
endif(CGOGN_BUILD_TESTS)

#### Endianness detection
include (TestBigEndian)
test_big_endian(CGOGN_TEST_BIG_ENDIAN)

#### Link time optimisation
if (CGOGN_ENABLE_LTO AND ("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU") AND (NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Debug"))
	add_flags(CMAKE_CXX_FLAGS_RELEASE -flto)
	find_program(GCC_AR gcc-ar)
	if (GCC_AR)
	set(CMAKE_AR ${GCC_AR})
	endif()
	find_program(GCC_RANLIB gcc-ranlib)
	if (GCC_RANLIB)
	set(CMAKE_RANLIB ${GCC_RANLIB})
	endif()
endif()

include(GenerateExportHeader)
include(CMakePackageConfigHelpers)


subdirlist(CGOGN_CONFIGURED_MODULES ${CGOGN_SOURCE_DIR}/cgogn)

add_subdirectory(${CGOGN_THIRDPARTY_DIR})

foreach(subdir ${CGOGN_CONFIGURED_MODULES})
	set(enabled_by_default ON)
	option( MODULE_${subdir} "Enable CGOGN component ${subdir}" ${enabled_by_default})
	if(MODULE_${subdir})
	list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cgogn/${subdir}")
	endif()
endforeach()

foreach(subdir ${CGOGN_CONFIGURED_MODULES})
	if(MODULE_${subdir})
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/cgogn/${subdir})
	endif()
endforeach()

foreach(subdir ${CGOGN_CONFIGURED_MODULES})
	if(CGOGN_BUILD_TESTS)
	if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cgogn/${subdir}/tests/)
		add_subdirectory(cgogn/${subdir}/tests)
	endif()
	endif()
	if(CGOGN_BUILD_EXAMPLES)
	if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cgogn/${subdir}/examples/)
		add_subdirectory(cgogn/${subdir}/examples)
	endif()
	endif()
endforeach()

if(${CGOGN_BUILD_BENCHS})
	add_subdirectory(benchmarks)
endif(${CGOGN_BUILD_BENCHS})


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
