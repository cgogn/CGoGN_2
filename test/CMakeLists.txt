
aux_source_directories(SOURCES "" .)

add_executable(test_chunk_array ${SOURCES})

target_link_libraries(test_chunk_array cgogn)
