#!/bin/sh


echo "-------------- Checking for Cmake --------------"

if (cmake --version) ; then
	echo "CMake found"
	echo
else
	echo "Error: CMake not found, please install it (see http://www.cmake.org)"
	exit 1
fi

usage="usage : ./configure.sh build-platform"


# Check the current OS
os="$1"
if [ -z "$os"] #if os is empty
	then 
		os=`uname -a`
fi		

case "$os" in
	Linux*x86_64*)
		os=Linux64-gcc;;
	Linux*amd64*)
		os=Linux64-gcc;;
	Darwin)
		os=Darwin-clang;;
	*)
		echo "Error: OS not supported: $os"			
		exit 1;;
esac	

# Generate the Makefiles
for config in Release Debug
do
	platform=$os-$config
	echo
	echo "-------------- Checking makefiles for $config mode --------------"
	echo

	mkdir -p build/$platform
	(cd build/$platform; cmake -DCMAKE_BUILD_TYPE:STRING=$config -DCGoGN_PLATFORM:STRING=$os ../../)
done

echo
echo "-------------- CGoGN build configured --------------"
echo
echo "to build CGoGN:
- go to build/$os-Release or build/$os-Debug
- and 'make' or 'cmake --build .'"

exit $?