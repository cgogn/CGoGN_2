
aux_source_directories(SOURCES "Source Files" .)
aux_source_directories(SOURCES "Source Files\\container" container)
aux_source_directories(SOURCES "Source Files\\map" container)
aux_source_directories(SOURCES "Source Files\\basic" basic)

include_directories(${CMAKE_BINARY_DIR}/src/lib)

add_library(cgogn ${SOURCES} )

