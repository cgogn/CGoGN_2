cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cgogn_modeling_examples
	LANGUAGES CXX
)


find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(cgogn_modeling REQUIRED)


set(CGOGN_TEST_PREFIX "test_")
set(CGOGN_TEST_MESHES_PATH "${CMAKE_SOURCE_DIR}/data/meshes/")
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_executable(triangular_tiling triangular_tiling.cpp)
target_link_libraries(triangular_tiling cgogn::core cgogn::io cgogn::geometry cgogn::modeling)

add_executable(square_tiling square_tiling.cpp)
target_link_libraries(square_tiling cgogn::core cgogn::io cgogn::geometry cgogn::modeling)

if (CGOGN_USE_QT)
find_package(cgogn_rendering REQUIRED)
find_package(QOGLViewer REQUIRED)

add_executable(remeshing remeshing.cpp)
target_link_libraries(remeshing cgogn::core cgogn::io cgogn::geometry cgogn::modeling cgogn::rendering)

add_executable(decimation decimation.cpp)
target_link_libraries(decimation cgogn::core cgogn::io cgogn::geometry cgogn::modeling cgogn::rendering)

add_executable(subdivision subdivision.cpp)
target_link_libraries(subdivision cgogn::core cgogn::io cgogn::geometry cgogn::modeling cgogn::rendering)

add_executable(dual dual.cpp)
target_link_libraries(dual cgogn::core cgogn::io cgogn::geometry cgogn::modeling cgogn::rendering)

if (Qt5Widgets_VERSION VERSION_GREATER 5.5.99)
  add_executable(curve curve.cpp)
  target_link_libraries(curve cgogn::core cgogn::io cgogn::geometry cgogn::modeling cgogn::rendering)
  set_target_properties(curve PROPERTIES FOLDER examples/modeling)
endif()


set_target_properties(remeshing decimation subdivision triangular_tiling square_tiling dual
PROPERTIES FOLDER examples/modeling)

endif()
