set(CGOGN_THIRDPARTY_QOGLVIEWER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "QOGLViewer include directory")

project(QOGLViewer)
find_package(OpenGL REQUIRED)
find_package(Qt5 5.4.0 COMPONENTS Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)

# in order to be able to include generated files (ui and moc)
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(HEADER_FILES
	QOGLViewer/qoglviewer.h
	QOGLViewer/camera.h
	QOGLViewer/manipulatedFrame.h
	QOGLViewer/manipulatedCameraFrame.h
	QOGLViewer/frame.h
	QOGLViewer/keyFrameInterpolator.h
	QOGLViewer/config.h
	QOGLViewer/constraint.h
	QOGLViewer/mouseGrabber.h
	QOGLViewer/quaternion.h
	QOGLViewer/vec.h
)

set(SOURCE_FILES
	QOGLViewer/qoglviewer.cpp
	QOGLViewer/camera.cpp
	QOGLViewer/manipulatedFrame.cpp
	QOGLViewer/manipulatedCameraFrame.cpp
	QOGLViewer/frame.cpp
	QOGLViewer/saveSnapshot.cpp
	QOGLViewer/constraint.cpp
	QOGLViewer/keyFrameInterpolator.cpp
	QOGLViewer/mouseGrabber.cpp
	QOGLViewer/quaternion.cpp
	QOGLViewer/vec.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARY} ${Qt5Widgets_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${Qt5Widgets_INCLUDE_DIRS})

if(WIN32)
	add_definitions(-DCREATE_QGLVIEWER_DLL -DNOMINMAX /W3)
else()
	target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
	add_definitions("-fPIC")
endif()

# for glu warning on mac
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	add_flags(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations")
endif()

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
	DESTINATION "include/thirdparty/"
	FILES_MATCHING PATTERN "*.h"
)

cgogn_create_package("${CMAKE_CURRENT_SOURCE_DIR}" "include")
