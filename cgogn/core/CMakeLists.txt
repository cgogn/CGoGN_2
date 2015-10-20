project(cgogn_core
		LANGUAGES CXX)

set(HEADER_FILES
	basic/definitions.h
	basic/nameTypes.h

	container/chunk_array_container.h
	container/chunk_array_factory.h
	container/chunk_array_gen.h
	container/chunk_array.h
	container/chunk_heap.h
	)

set(SOURCE_FILES
	container/chunk_array.cpp
	)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
#target_compile_features(${PROJECT_NAME} PUBLIC
#												cxx_nullptr
#												cxx_range_for
#												cxx_static_assert
#												cxx_deleted_functions
#												cxx_defaulted_functions
#												cxx_auto_type
#												cxx_alias_templates
#												cxx_final
#												cxx_defaulted_move_initializers
#												cxx_default_function_template_args
#												cxx_decltype
#												cxx_lambdas
#												cxx_override
#												cxx_return_type_deduction
#												cxx_strong_enums
#												cxx_template_template_parameters
#							)

target_include_directories(${PROJECT_NAME} PUBLIC
					$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
  					$<INSTALL_INTERFACE:include/cgogn/cgogn_core>
						  )

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




