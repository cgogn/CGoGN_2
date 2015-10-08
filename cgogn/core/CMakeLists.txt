
aux_source_directories(SOURCES "Source Files" .)
aux_source_directories(SOURCES "Source Files\\container" container)

include_directories(${CMAKE_BINARY_DIR}/src/lib)

add_library(cgogn ${SOURCES} )

