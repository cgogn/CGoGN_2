#!/bin/bash
if [[ $TRAVIS_OS_NAME == 'osx' ]]; then
 brew update
 brew install cmake
else
 sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
 sudo apt-get update
 sudo apt-get install "g++-5"
 sudo apt-get install cmake3
fi

