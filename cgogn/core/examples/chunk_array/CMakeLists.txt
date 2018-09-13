project(${CGOGN_TEST_PREFIX}chunk_array
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)


add_executable(chunk_array chunk_array.cpp)
target_link_libraries(chunk_array cgogn::core)

add_executable(bench_chunk_array bench_chunk_array.cpp)
target_link_libraries(bench_chunk_array cgogn::core)

add_executable(chunk_array2 chunk_array2.cpp)
target_link_libraries(chunk_array2 cgogn::core)

set_target_properties(chunk_array bench_chunk_array chunk_array2 PROPERTIES FOLDER examples/core)
