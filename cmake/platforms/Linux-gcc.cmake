#-------------------------------------------------------------------
# Flags common to all Linux based platforms with GNU compiler
#-------------------------------------------------------------------

include(${CMAKE_SOURCE_DIR}/cmake/platforms/Linux.cmake)

# Warning flags
set(NORMAL_WARNINGS -Wall -Wextra)
set(FULL_WARNINGS
    ${NORMAL_WARNINGS}
    -pedantic
    -Wno-long-long
    -Wconversion
    -Winline
)

# Determine gcc version and activate additional warnings available in latest versions
execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)

if (GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
    message(STATUS "GCC version >= 4.3, activating sign conversion warnings")
    set(FULL_WARNINGS ${FULL_WARNINGS} -Wsign-conversion)
endif()

if (GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
    message(STATUS "GCC version >= 4.6, activating double promotion warnings")
    set(FULL_WARNINGS ${FULL_WARNINGS} -Wdouble-promotion)
endif()


# Compile with full warnings by default
add_definitions(${FULL_WARNINGS})

# Warn about missing virtual destructor (C++ only)
add_flags(CMAKE_CXX_FLAGS -Wnon-virtual-dtor)

# Add static and dynamic bounds checks (optimization required)
if (GCC_VERSION VERSION_GREATER 4.0)
   add_flags(CMAKE_CXX_FLAGS_RELEASE -D_FORTIFY_SOURCE=2)
   add_flags(CMAKE_C_FLAGS_RELEASE -D_FORTIFY_SOURCE=2)
endif()

# Enable SSE3 instruction set
add_flags(CMAKE_CXX_FLAGS -msse3)
add_flags(CMAKE_C_FLAGS -msse3)

# Always generate position independant code
# (to allow linking with DLLs)
add_flags(CMAKE_CXX_FLAGS -fPIC)
add_flags(CMAKE_C_FLAGS -fPIC)

# Profiler compilation flags
if(CGOGN_USE_GPROF)
    message(STATUS "Building for code profiling")
    add_flags(CMAKE_CXX_FLAGS -pg -DPROFILER)
    add_flags(CMAKE_C_FLAGS -pg -DPROFILER)
endif(CGOGN_USE_GPROF)

# Code coverage compilation flags
if(CGOGN_USE_GCOV)
    message(STATUS "Building for coverage analysis")
    add_flags(CMAKE_CXX_FLAGS --coverage)
    add_flags(CMAKE_C_FLAGS --coverage)
endif(CGOGN_USE_GCOV)

# Compilation flags for Google's AddressSanitizer
# These flags can only be specified for dynamic builds
if(CGOGN_USE_ASAN)
    message(STATUS "Building with AddressSanitizer (debug only)")
    add_flags(CMAKE_CXX_FLAGS_DEBUG -fsanitize=address -fno-omit-frame-pointer)
    add_flags(CMAKE_C_FLAGS_DEBUG -fsanitize=address -fno-omit-frame-pointer)
endif(CGOGN_USE_ASAN)
#TODO Use native GCC stack smash Protection and buffer overflow detection in debug when no asan ??

# Compilation flags for Google's ThreadSanitizer
# Does not work for the moment: cannot figure out how to link with library libtsan
if(CGOGN_USE_TSAN)
    message(STATUS "Building with ThreadSanitizer (debug only)")
    add_flags(CMAKE_CXX_FLAGS_DEBUG -fsanitize=thread)
    add_flags(CMAKE_C_FLAGS_DEBUG -fsanitize=thread)
endif(CGOGN_USE_TSAN)



macro(m_add_executable)
    
    # Create a statically linked executable
    # Link with static libraries
    add_flags(CMAKE_CXX_FLAGS -static-libstdc++ -static-libgcc )
    add_flags(CMAKE_C_FLAGS -static-libgcc -static)
    
    add_executable(${ARGN})
endmacro()

