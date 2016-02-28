set(CGOGN_THIRDPARTY_QOGLVIEWER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "QOGLViewer include directory")


PROJECT(QOGLViewer)
FIND_PACKAGE(OpenGL REQUIRED)
find_package(Qt5Widgets 5.4.0 REQUIRED)

set(CMAKE_AUTOMOC ON)

# in order to be able to include generated files (ui and moc)
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(HEADER_FILES
	qoglviewer.h
	camera.h 
	manipulatedFrame.h 
	manipulatedCameraFrame.h 
	frame.h 
	keyFrameInterpolator.h 	

	config.h
	constraint.h 
	mouseGrabber.h 
	quaternion.h 
	vec.h 
	)
	
set(SOURCE_FILES
	qoglviewer.cpp
	camera.cpp 
	manipulatedFrame.cpp 
	manipulatedCameraFrame.cpp 
	frame.cpp 
	saveSnapshot.cpp 
	constraint.cpp 
	keyFrameInterpolator.cpp 
	mouseGrabber.cpp 
	quaternion.cpp 
	vec.cpp
	)

if(WIN32)
    ADD_DEFINITIONS(-DCREATE_QGLVIEWER_DLL -DNOMINMAX /W3)
else()
	ADD_DEFINITIONS(-fPIC)
endif() 


add_library(QOGLViewer SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(QOGLViewer  ${OPENGL_LIBRARY} Qt5::Widgets)



