#!/bin/bash
if [[ $TRAVIS_OS_NAME == 'linux' ]]; then
 sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
 sudo apt-get update
 sudo apt-get install "g++-5"
 sudo apt-get install cmake3
 sudo apt-get install libglu1-mesa-dev
 pushd /opt/
 wget https://mycore.core-cloud.net/index.php/s/dgC98J3K4YqGF7i/download -O qt56linux.tgz 
 tar xf qt56linux.tgz
 popd
fi


