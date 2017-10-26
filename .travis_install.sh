#!/bin/bash
if [[ $TRAVIS_OS_NAME == 'linux' ]]; then
 sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
 sudo apt-get update
 sudo apt-get install "g++-5"
 sudo apt-get install cmake3
 sudo apt-get install libglu1-mesa-dev
fi

