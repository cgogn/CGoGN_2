# Allow the source files to find headers in src/
include_directories(${PROJECT_SOURCE_DIR}/src)

# Define the source files
set(SOURCE_FILES "benchmark.cc" "colorprint.cc" "commandlineflags.cc"
                 "console_reporter.cc" "csv_reporter.cc" "json_reporter.cc"
                 "log.cc" "reporter.cc" "sleep.cc" "string_util.cc"
                 "sysinfo.cc" "walltime.cc")
# Determine the correct regular expression engine to use
if(HAVE_STD_REGEX)
	set(RE_FILES "re_std.cc")
elseif(HAVE_GNU_POSIX_REGEX)
	set(RE_FILES "re_posix.cc")
elseif(HAVE_POSIX_REGEX)
	set(RE_FILES "re_posix.cc")
else()
	message(FATAL_ERROR "Failed to determine the source files for the regular expression backend")
endif()

add_library(benchmark ${SOURCE_FILES} ${RE_FILES})
# use of target_compile_options to have a transitive c++11 flag
if(NOT MSVC)
	target_compile_options(benchmark PUBLIC "-std=c++11")
endif()

set_target_properties(benchmark PROPERTIES
	OUTPUT_NAME "benchmark"
	VERSION ${GENERIC_LIB_VERSION}
	SOVERSION ${GENERIC_LIB_SOVERSION}
)

# Link threads.
target_link_libraries(benchmark ${CMAKE_THREAD_LIBS_INIT})

# We need extra libraries on Windows
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	target_link_libraries(benchmark Shlwapi)
endif()

# Expose public API
target_include_directories(benchmark PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/thirdparty>)

install(
	DIRECTORY "${PROJECT_SOURCE_DIR}/include/benchmark"
	DESTINATION "include/thirdparty/"
	FILES_MATCHING PATTERN "*.*h")

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "include")
