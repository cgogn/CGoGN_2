project(cgogn_utils
        LANGUAGES CXX)

set(HEADER_FILES
    allocator.h
    buffers.h
    make_unique.h
    )

set(SOURCE_FILES
    allocator.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
#use of target_compile_options to have a transitive c++11 flag
if(NOT MSVC)
    target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
                                        $<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
                                        $<INSTALL_INTERFACE:include/cgogn/utils>
                                                  )
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DESTINATION include/cgogn/utils
        FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
           )
