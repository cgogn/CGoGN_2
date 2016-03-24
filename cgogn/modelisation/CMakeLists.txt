project(cgogn_modelisation
	LANGUAGES CXX
)

set(HEADER_FILES
	dll.h
	algos/catmull_clark.h
)

 add_custom_target(cgogn_modelisation SOURCES ${HEADER_FILES})
#add_library(HEADER_ONLY_TARGET ${PROJECT_NAME} ${HEADER_FILES})
#set_target_properties(HEADER_ONLY_TARGET PROPERTIES LINKER_LANGUAGE CXX)

install(DIRECTORY .
	DESTINATION include/cgogn/modelisation
	FILES_MATCHING PATTERN "*.h"
)

if(CGOGN_BUILD_TESTS)
	add_subdirectory(tests)
endif()
