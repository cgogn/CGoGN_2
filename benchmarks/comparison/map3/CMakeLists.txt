cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(bench_comparison3
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
	set(CGOGN1_INCLUDE_DIRS ${CGOGN1_PATH}/CGoGN/include ${CGOGN1_PATH}/ThirdParty ${CGOGN1_PATH}/ThirdParty/TinyXml2 ${CGOGN1_PATH}/ThirdParty/OpenNL/src)
	list(APPEND CPP_FILES cgogn1/cgogn_performance_3.cpp)
	list(APPEND HEADER_FILES cgogn1/cgogn_performance_3.h)
	find_library(CGOGN1_ALGO_LIB NAMES algo HINTS ${CGOGN1_PATH}/lib/${CMAKE_BUILD_TYPE})
	find_library(CGOGN1_CONTAINER_LIB NAMES container HINTS ${CGOGN1_PATH}/lib/${CMAKE_BUILD_TYPE})
	find_library(CGOGN1_TOPO_LIB NAMES topology HINTS ${CGOGN1_PATH}/lib/${CMAKE_BUILD_TYPE})
	find_library(CGOGN1_UTILS_LIB NAMES utils HINTS ${CGOGN1_PATH}/lib/${CMAKE_BUILD_TYPE})
	find_library(CGOGN1_NL_LIB NAMES nl HINTS ${CGOGN1_PATH}/lib/${CMAKE_BUILD_TYPE})
	set(CGOGN1_LIBRARIES ${CGOGN1_ALGO_LIB} ${CGOGN1_CONTAINER_LIB} ${CGOGN1_TOPO_LIB} ${CGOGN1_UTILS_LIB} ${CGOGN1_NL_LIB})
endif()

set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "preserve cmake flags" FORCE)
find_package(CGAL QUIET)
if(CGAL_FOUND)
	include(${CGAL_USE_FILE})
	list(APPEND HEADER_FILES cgal/lcc_performance_3.h)
	list(APPEND CPP_FILES cgal/lcc_performance_3.cpp)
else()
	message(STATUS "bench_comparison3: not using CGAL (not found)")
endif()


find_package(OpenVolumeMesh)
if(OPENVOLUMEMESH_FOUND)
	list(APPEND HEADER_FILES openvolumemesh/openvolumemesh_performance.h)
	list(APPEND CPP_FILES openvolumemesh/openvolumemesh_performance.cpp)
else()
	message(STATUS "bench_comparison3: not using OpenVolumeMesh (not found)")
endif()

list(APPEND CPP_FILES performance.cpp)
list(APPEND HEADER_FILES performance.h)
list(APPEND CPP_FILES cgogn2/cgogn2_performance_3.cpp)
list(APPEND HEADER_FILES cgogn2/cgogn2_performance_3.h)

add_executable(${PROJECT_NAME} ${CPP_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_SOURCE_DIR}/thirdparty/google-benchmark/include
	)

target_link_libraries(${PROJECT_NAME}
			${cgogn_core_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_geometry_LIBRARIES} ${cgogn_modeling_LIBRARIES}
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

if(OPENVOLUMEMESH_FOUND)
	target_link_libraries(${PROJECT_NAME} ${OPENVOLUMEMESH_LIBRARIES})
	target_include_directories(${PROJECT_NAME} PRIVATE ${OPENVOLUMEMESH_INCLUDE_DIR})
endif()


set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER benchmarks)