cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(bench_comparison2
	LANGUAGES CXX
	)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

find_package(cgogn_core REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(cgogn_modeling REQUIRED)
find_package(benchmark REQUIRED)

set(CGOGN1_PATH "" CACHE PATH "Path to CGoGN.")

set(CGOGN1_INCLUDE_DIRS "")
set(CGOGN1_LIBRARIES "")
if (CGOGN1_PATH)
	set(CGOGN1_INCLUDE_DIRS ${CGOGN1_PATH}/CGoGN/include ${CGOGN1_PATH}/ThirdParty ${CGOGN1_PATH}/ThirdParty/TinyXml2)
	list(APPEND CPP_FILES cgogn1/cgogn1_performance.cpp)
	list(APPEND HEADER_FILES cgogn1/cgogn1_performance.h)
	find_library(CGOGN1_ALGO_LIB NAMES algo HINTS ${CGOGN1_PATH}/lib/${CMAKE_BUILD_TYPE})
	find_library(CGOGN1_CONTAINER_LIB NAMES container HINTS ${CGOGN1_PATH}/lib/${CMAKE_BUILD_TYPE})
	find_library(CGOGN1_TOPO_LIB NAMES topology HINTS ${CGOGN1_PATH}/lib/${CMAKE_BUILD_TYPE})
	find_library(CGOGN1_UTILS_LIB NAMES utils HINTS ${CGOGN1_PATH}/lib/${CMAKE_BUILD_TYPE})
	set(CGOGN1_LIBRARIES ${CGOGN1_ALGO_LIB} ${CGOGN1_CONTAINER_LIB} ${CGOGN1_TOPO_LIB} ${CGOGN1_UTILS_LIB})
endif()

set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "preserve cmake flags" FORCE)
find_package(CGAL QUIET)
if(CGAL_FOUND)
	include(${CGAL_USE_FILE})
	list(APPEND HEADER_FILES cgal/lcc_performance_2.h)
	list(APPEND HEADER_FILES cgal/polyhedron_performance.h)
	list(APPEND CPP_FILES cgal/lcc_performance_2.cpp)
	list(APPEND CPP_FILES cgal/polyhedron_performance.cpp)
else()
	message(STATUS "bench_comparison2: not using CGAL (not found)")
endif()


find_package(OpenMesh QUIET)
if(OPENMESH_FOUND)
	list(APPEND HEADER_FILES openmesh/openmesh_performance.h)
	list(APPEND CPP_FILES openmesh/openmesh_performance.cpp)
else()
	message(STATUS "bench_comparison2: not using OpenMesh (not found)")
endif()

add_subdirectory(surface_mesh/surface_mesh)
list(APPEND CPP_FILES performance.cpp)
list(APPEND CPP_FILES surface_mesh/surface_mesh_performance.cpp)
list(APPEND CPP_FILES cgogn2/cgogn2_performance.cpp)
list(APPEND HEADER_FILES performance.h)
list(APPEND HEADER_FILES cgogn2/cgogn2_performance.h)

add_executable(${PROJECT_NAME} ${CPP_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_SOURCE_DIR}/thirdparty/google-benchmark/include
	)

target_link_libraries(${PROJECT_NAME}
			${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_geometry_LIBRARIES} ${cgogn_modeling_LIBRARIES}
			surface_mesh
			${benchmark_LIBRARIES}
			)

if (CGOGN1_PATH)
	target_link_libraries(${PROJECT_NAME} ${CGOGN1_LIBRARIES})
	target_include_directories(${PROJECT_NAME} PRIVATE ${CGOGN1_INCLUDE_DIRS})
endif()

if(CGAL_FOUND)
	target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES})
	target_include_directories(${PROJECT_NAME} PRIVATE ${CGAL_INCLUDE_DIRS})
endif()

if(OPENMESH_FOUND)
	target_link_libraries(${PROJECT_NAME} ${OPENMESH_LIBRARIES})
	target_include_directories(${PROJECT_NAME} PRIVATE ${OPENMESH_INCLUDE_DIR})
endif()


set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER benchmarks)
set_target_properties(surface_mesh PROPERTIES FOLDER benchmarks)