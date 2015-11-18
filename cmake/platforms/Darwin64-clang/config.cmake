include(${CMAKE_SOURCE_DIR}/cmake/platforms/Darwin-clang.cmake)
string_append(CMAKE_CXX_FLAGS -m64)
string_append(CMAKE_C_FLAGS -m64)


