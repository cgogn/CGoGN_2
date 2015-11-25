project(${CGOGN_TEST_PREFIX}geometry
		LANGUAGES CXX
	   )

#get_property(cgogn_core_includes
#			  TARGET cgogn_core
#			  PROPERTY INCLUDE_DIRECTORIES
#			)

add_executable(test_geometry test_geometry.cpp)
target_link_libraries(test_geometry cgogn_core)
target_include_directories(test_geometry PUBLIC
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/cgogn/cgogn_core>
)

install(TARGETS test_geometry
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "bin/test"
           )

