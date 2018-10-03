
if(CGOGN_USE_QT)
	add_subdirectory(libQGLViewer)
	set_target_properties(QOGLViewer PROPERTIES FOLDER thirdparty)
endif(CGOGN_USE_QT)

if (CGOGN_BUILD_BENCHS)
	add_subdirectory(google-benchmark)
	set_target_properties(benchmark PROPERTIES FOLDER thirdparty)
endif(CGOGN_BUILD_BENCHS)

add_subdirectory(ply)
add_subdirectory(OffBinConverter)
add_subdirectory(libMeshb)
add_subdirectory(termcolor)
