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
string_append(CMAKE_CXX_FLAGS_RELEASE -D_FORTIFY_SOURCE=2)
string_append(CMAKE_C_FLAGS_RELEASE -D_FORTIFY_SOURCE=2)

# Enable SSE3 instruction set
string_append(CMAKE_CXX_FLAGS -msse3)
string_append(CMAKE_C_FLAGS -msse3)

# Additional debug flags
string_append(CMAKE_CXX_FLAGS_DEBUG -D_GLIBCXX_DEBUG)


string_append(CMAKE_CXX_FLAGS -std=c++11)

macro(m_add_executable)
    
    # Create a statically linked executable
    # Link with static libraries
    string_append(CMAKE_CXX_FLAGS -static-libstdc++ -static-libgcc )
    string_append(CMAKE_C_FLAGS -static-libgcc -static)
    
    add_executable(${ARGN})
endmacro()

