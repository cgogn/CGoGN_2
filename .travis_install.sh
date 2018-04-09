#!/bin/bash
if [[ $TRAVIS_OS_NAME == 'linux' ]]; then
 sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
 sudo apt-get update
 sudo apt-get install "g++-5"
 sudo apt-get install libglu1-mesa-dev
 CMAKE_URL="https://cmake.org/files/v3.7.2/cmake-3.7.2-Linux-x86_64.tar.gz"
 mkdir cmake && travis_retry wget --no-check-certificate --quiet -O - ${CMAKE_URL} | tar --strip-components=1 -xz -C cmake
 export PATH=${DEPS_DIR}/cmake/bin:${PATH}
 cmake --version
 pushd /opt/
 wget https://mycore.core-cloud.net/index.php/s/dgC98J3K4YqGF7i/download -O qt56linux.tgz 
 tar xf qt56linux.tgz
 popd
fi


