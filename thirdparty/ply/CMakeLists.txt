set(CGOGN_THIRDPARTY_PLY_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Ply include directory")

project(ply
	LANGUAGES C
	)

set(HEADER_FILES
	ply.h
	)

set(SOURCE_FILES
	ply.c
	)

add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")


######## 1. Build tree

export(TARGETS ${PROJECT_NAME}
	FILE "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Targets.cmake")

set(PLY_INCLUDE_DIR "${CGOGN_SOURCE_DIR}")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake"
	PATH_VARS PLY_INCLUDE_DIR
	INSTALL_DESTINATION "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}"
)

## <package_name>ConfigVersion.cmake
write_basic_package_version_file(
	"${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake"
	VERSION ${CGOGN_VERSION_MAJOR}.${CGOGN_VERSION_MINOR}.${CGOGN_VERSION_PATCH}
	COMPATIBILITY ExactVersion
)

######## 2. Install tree

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION "bin"
	LIBRARY DESTINATION "lib"
	ARCHIVE DESTINATION "lib"
)
install(EXPORT ${PROJECT_NAME}Targets DESTINATION "lib/cmake/${PROJECT_NAME}")

## <package_name>ConfigVersion.cmake
write_basic_package_version_file(
	"${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake"
	VERSION ${CGOGN_VERSION_MAJOR}.${CGOGN_VERSION_MINOR}.${CGOGN_VERSION_PATCH}
	COMPATIBILITY ExactVersion
)

## <package_name>Config.cmake
set(PLY_INCLUDE_DIR "include")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake"
	PATH_VARS PLY_INCLUDE_DIR
	INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
)

install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}")
install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}" RENAME "${PROJECT_NAME}Config.cmake")
