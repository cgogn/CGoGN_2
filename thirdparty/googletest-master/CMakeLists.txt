cmake_minimum_required(VERSION 3.0)

project( googletest-distribution )

cgogn_reset_warning_level()


enable_testing()

option(BUILD_GTEST "Builds the googletest subproject" OFF)

#Note that googlemock target already builds googletest
option(BUILD_GMOCK "Builds the googlemock subproject" ON)

if (MSVC AND MSVC_VERSION GREATER 1800)
  add_definitions(/D _SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING)
endif()

if(BUILD_GMOCK)
  add_subdirectory( googlemock )
elseif(BUILD_GTEST)
  add_subdirectory( googletest )
endif()

