if(CGOGN_PROVIDE_EIGEN)
	add_subdirectory(eigen-3.2.9)
endif(CGOGN_PROVIDE_EIGEN)

if(CGOGN_PROVIDE_TINYXML2)
	add_subdirectory(TinyXml2)
endif(CGOGN_PROVIDE_TINYXML2)

if(CGOGN_BUILD_TESTS)
	add_subdirectory(googletest-master)
endif(CGOGN_BUILD_TESTS)

if(CGOGN_USE_QT)
	add_subdirectory(libQGLViewer)
endif(CGOGN_USE_QT)

if (CGOGN_BUILD_BENCHS)
	add_subdirectory(google-benchmark)
endif(CGOGN_BUILD_BENCHS)

add_subdirectory(ply)
add_subdirectory(OffBinConverter)
add_subdirectory(lm6)
add_subdirectory(termcolor)
add_subdirectory(zlib-1.2.8)
