project(cgogn_multiresolution
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)

set(HEADER_FILES
	dll.h

	cph/attribute_handler_cph.h
	cph/cph_base.h
	cph/cph2.h
	cph/cph3.h
	cph/ihcmap2.h
	cph/ihcmap2_adaptive.h
	cph/ihcmap2_regular.h
	cph/ihcmap3.h

	mrcmap/mr_base.h
	mrcmap/mrcmap2.h

#	mra/mr_analysis.h
#	mra/lerp_triquad_mra.h
)

set(SOURCE_FILES
	cph/cph_base.cpp
	cph/cph2.cpp
	cph/cph3.cpp

	cph/ihcmap2.cpp
	cph/ihcmap2_adaptive.cpp
	cph/ihcmap2_regular.cpp
	cph/ihcmap3.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_geometry_LIBRARIES})

install(FILES "dll.h" DESTINATION "include/cgogn/multiresolution")
install(DIRECTORY cph rcmap
	DESTINATION include/cgogn/multiresolution
	FILES_MATCHING PATTERN "*.h"
)

######## 1. Build tree

export(TARGETS ${PROJECT_NAME}
	FILE "${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Targets.cmake")

set(CGOGN_MULTIRESOLUTION_INCLUDE_DIRS "${CGOGN_SOURCE_DIR}")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake"
	PATH_VARS CGOGN_MULTIRESOLUTION_INCLUDE_DIRS
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
set(CGOGN_MULTIRESOLUTION_INCLUDE_DIRS "include")
configure_package_config_file(
	"${CGOGN_PATH}/cmake/ConfigFiles/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake"
	PATH_VARS CGOGN_MULTIRESOLUTION_INCLUDE_DIRS
	INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
)

install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}")
install(FILES "${CMAKE_BINARY_DIR}/share/cmake/${PROJECT_NAME}/${PROJECT_NAME}InstallConfig.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}" RENAME "${PROJECT_NAME}Config.cmake")

#add_subdirectory(examples)
