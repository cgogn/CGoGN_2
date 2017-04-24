project(surface_mesh
	LANGUAGES CXX
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(${PROJECT_NAME} IO.cpp IO_off.cpp Surface_mesh.cpp)
