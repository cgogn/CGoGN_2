project(surface_mesh
	LANGUAGES CXX
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if (WIN32)
  add_library(${PROJECT_NAME} STATIC IO.cpp IO.h IO_off.cpp Surface_mesh.cpp Surface_mesh.h)
else()
  add_library(${PROJECT_NAME} IO.cpp IO.h IO_off.cpp Surface_mesh.cpp Surface_mesh.h)
  target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
endif()
