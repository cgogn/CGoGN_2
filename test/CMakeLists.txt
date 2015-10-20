
aux_source_directories(SOURCES "" .)

add_executable(test_chunk_array test_chunk_array.cpp)
target_link_libraries(test_chunk_array cgogn)

add_executable(bench_chunk_array bench_chunk_array.cpp)
target_link_libraries(bench_chunk_array cgogn)
