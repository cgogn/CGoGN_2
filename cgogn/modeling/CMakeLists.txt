project(cgogn_modeling
	LANGUAGES CXX
)

set(HEADER_FILES
	dll.h
	algos/catmull_clark.h
	algos/pliant_remeshing.h
	algos/loop.h
)

add_custom_target(cgogn_modeling SOURCES ${HEADER_FILES})
#add_library(HEADER_ONLY_TARGET ${PROJECT_NAME} ${HEADER_FILES})
#set_target_properties(HEADER_ONLY_TARGET PROPERTIES LINKER_LANGUAGE CXX)

install(FILES "dll.h" DESTINATION "include/cgogn/modeling")
install(DIRECTORY algos
	DESTINATION include/cgogn/modeling
	FILES_MATCHING PATTERN "*.h"
)
