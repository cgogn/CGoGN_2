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
string_append(CMAKE_CXX_FLAGS -Wnon-virtual-dtor)

# Add static and dynamic bounds checks (optimization required)
if (GCC_VERSION VERSION_GREATER 4.0)
   string_append(CMAKE_CXX_FLAGS_RELEASE -D_FORTIFY_SOURCE=2)
   string_append(CMAKE_C_FLAGS_RELEASE -D_FORTIFY_SOURCE=2)
endif()

# Enable SSE3 instruction set
string_append(CMAKE_CXX_FLAGS -msse3)
string_append(CMAKE_C_FLAGS -msse3)

# Enable glibc parallel mode
#add_flags(CMAKE_CXX_FLAGS -D_GLIBCXX_PARALLEL)

# Enable some algebraic transforms
# (necessary for vectorizing certain reductions and dot products)
#!!! -- deactivated, because it breaks Shewchuck's predicates --
#add_flags(CMAKE_CXX_FLAGS -ffast-math -fassociative-math)
#add_flags(CMAKE_C_FLAGS -ffast-math -fassociative-math)
#-ftree-vectorizer-verbose=2 

# Generate debug information even in release mode
#add_flags(CMAKE_CXX_FLAGS_RELEASE -g)
#add_flags(CMAKE_C_FLAGS_RELEASE -g)


# Additional debug flags
# deactivated for now: I added bound checking in VOR::vector<>.
#add_flags(CMAKE_CXX_FLAGS_DEBUG -D_GLIBCXX_DEBUG)


# Compile and link with OpenMP
#if (GCC_VERSION VERSION_GREATER 4.0)
#    string_append(CMAKE_CXX_FLAGS -fopenmp)
#    string_append(CMAKE_C_FLAGS -fopenmp)
#endif()

# Always generate position independant code
# (to allow linking with DLLs)
string_append(CMAKE_CXX_FLAGS -fPIC)
string_append(CMAKE_C_FLAGS -fPIC)


macro(m_add_executable)
    
    # Create a statically linked executable
    # Link with static libraries
    string_append(CMAKE_CXX_FLAGS -static-libstdc++ -static-libgcc )
    string_append(CMAKE_C_FLAGS -static-libgcc -static)
    
    add_executable(${ARGN})
endmacro()

