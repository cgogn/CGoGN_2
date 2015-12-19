project(cgogn_utils
	LANGUAGES CXX
)

set(HEADER_FILES
	assert.h
	buffers.h
	definitions.h
	dll.h
	make_unique.h
	name_types.h
	serialization.h
	thread.h
)

set(SOURCE_FILES
	assert.cpp
	thread.cpp
	name_types.cpp
	serialization.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

# use of target_compile_options to have a transitive c++11 flag
if(NOT MSVC)
	target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

# Eigen generated warnings
if(NOT MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include/cgogn/cgogn_utils>
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	DESTINATION include/cgogn/cgogn_utils
	FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)
