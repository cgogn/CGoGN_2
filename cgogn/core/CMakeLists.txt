project(cgogn_core
		LANGUAGES CXX)

set(HEADER_FILES
	basic/definitions.h
	basic/nameTypes.h
	basic/dart.h
	basic/cell.h
	basic/serialization.h
	container/chunk_array_container.h
	container/chunk_array_factory.h
	container/chunk_array_gen.h
	container/chunk_array.h
	container/chunk_stack.h

	map/map_base_data.h
	map/map_base.h
	map/map1.h
	map/map2.h
	map/map_tri.h

	map/attribute_handler.h
	)

set(SOURCE_FILES
	container/chunk_array_container.cpp
	)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
					$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
  					$<INSTALL_INTERFACE:include/cgogn/cgogn_core>
						  )
target_link_libraries(${PROJECT_NAME} cgogn_utils)

install(DIRECTORY basic container
        DESTINATION include/cgogn/cgogn_core
        FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
	   )




