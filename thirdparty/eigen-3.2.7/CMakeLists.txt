if (CGOGN_PROVIDE_EIGEN)
        set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Directory for find_package(Eigen3)")
        set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Include directory for find_package(Eigen3)")

        set(CGOGN_EIGEN_VERSION_STR "3.2.7")
        set(CGOGN_EIGEN_MD5 "76959f105cfbda3ba77889bc204f4bd2")
        set(CGOGN_OLD_EIGEN_DIRECTORY "eigen-eigen-b30b87236a1b")
        if(NOT EXISTS ${PROJECT_BINARY_DIR}/eigen${CGOGN_EIGEN_VERSION_STR}.tar.gz)
                file(DOWNLOAD http://bitbucket.org/eigen/eigen/get/${CGOGN_EIGEN_VERSION_STR}.tar.gz ${PROJECT_BINARY_DIR}/eigen${CGOGN_EIGEN_VERSION_STR}.tar.gz
                 TIMEOUT 180
                 SHOW_PROGRESS
                 EXPECTED_MD5 ${CGOGN_EIGEN_MD5}
                 )

                execute_process(
				WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E tar xzf eigen${CGOGN_EIGEN_VERSION_STR}.tar.gz
                )
        endif()
        if(NOT EXISTS ${EIGEN_ROOT}/Eigen)
                execute_process(
                WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_directory ${CGOGN_OLD_EIGEN_DIRECTORY}/Eigen ${EIGEN_ROOT}/Eigen
                )
                file(
                        GLOB_RECURSE to_remove
                        "${EIGEN_ROOT}/Eigen/CMakeLists.txt"
                        )
                file(REMOVE ${to_remove})
        endif()
endif()
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Eigen
        COMPONENT Eigen_headers
        DESTINATION include
        PATTERN "*.in" EXCLUDE
        PATTERN "*.txt" EXCLUDE
        PATTERN "*.cpp" EXCLUDE)