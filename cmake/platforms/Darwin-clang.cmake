#-------------------------------------------------------------------
# Flags common to all Darwin based platforms with CLANG compiler
#-------------------------------------------------------------------

include(${CMAKE_SOURCE_DIR}/cmake/platforms/Darwin.cmake)

#Warning flags
set(NORMAL_WARNINGS -Wall -Wextra)
set(FULL_WARNINGS
    -Weverything
    -Wno-unused-macros
    -Wno-disabled-macro-expansion
    -Wno-covered-switch-default
    -Wno-padded
    -Wno-float-equal
    # Ignore warnings about global variables ctors and dtors
    -Wno-global-constructors
    # Ignore warnings about global destructor 
    -Wno-exit-time-destructors
    # Turn this on to detect documentation errors (very useful)
    -Wno-documentation
    # Ignore unknown documentation command (There are nrecognized but valid doxygen commands !)
    -Wno-documentation-unknown-command
    # Too many of sign conversion problems. Ignore them for the moment.
    #-Wno-sign-
    # Ignore warnings about C++98 compatibility
    -Wno-c++98-compat
    # Ignore warnings about c++98 compat pedantic mode
    -Wno-c++98-compat-pedantic
    # Ignore warnings about C++11 extensions (cgogn is promoting c++11 )
    -Wno-c++11-extensions
)

# Compile with full warnings by default
add_definitions(${FULL_WARNINGS})

# Add static and dynamic bounds checks (optimization required)
add_flags(CMAKE_CXX_FLAGS_RELEASE -D_FORTIFY_SOURCE=2)
add_flags(CMAKE_C_FLAGS_RELEASE -D_FORTIFY_SOURCE=2)

# Enable SSE3 instruction set
add_flags(CMAKE_CXX_FLAGS -msse3)
add_flags(CMAKE_C_FLAGS -msse3)

# Additional debug flags
add_flags(CMAKE_CXX_FLAGS_DEBUG -D_GLIBCXX_DEBUG)

# Enable c++11 extensions
add_flags(CMAKE_CXX_FLAGS -std=c++11)

# Run clang static analyzer
# if(CGOGN_WITH_CLANGSA)
#     add_definitions(--analyze)
# endif(CGOGN_WITH_CLANGSA)

# Profiler compilation flags
if(CGOGN_USE_GPROF)
    message(FATAL_ERROR "Profiling is not (yet) available with clang")
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

