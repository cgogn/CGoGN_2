project(cgogn_modeling
	LANGUAGES CXX
)

find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)

set(HEADER_FILES
	dll.h
	algos/catmull_clark.h
	algos/pliant_remeshing.h
	algos/loop.h
        algos/refinements.h

        tiling/tiling.h
        tiling/triangular_grid.h
        tiling/triangular_cylinder.h
        tiling/triangular_tore.h
#        tiling/triangular_cube.h

        tiling/square_grid.h
        tiling/square_cylinder.h
        tiling/square_tore.h
#        tiling/square_cube.h
)

set(SOURCE_FILES
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_geometry_LIBRARIES})

install(FILES "dll.h" DESTINATION "include/cgogn/modeling")
install(DIRECTORY algos tiling
        DESTINATION "include/cgogn/modeling"
        FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CGOGN_SOURCE_DIR}" "include")
