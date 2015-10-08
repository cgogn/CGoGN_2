#!/bin/sh


echo "-------------- Checking for Cmake --------------"

if (cmake --version) ; then
	echo "CMake found"
	echo
else
	echo "Error: CMake not found, please install it (see http://www.cmake.org)"
	exit 1
fi


os="Linux64-gcc"
for config in Release Debug
do
	echo
	echo "-------------- Checking makefiles for $config mode --------------"
	echo

	mkdir -p build/$os-$config
	(cd build/$os-$config; cmake -DCMAKE_BUILD_TYPE:STRING=$config -DCGoGN_PLATFORM:STRING=$os ../../)
done

echo
echo "-------------- CGoGN build configured --------------"
echo

cat << EOF
to build CGoGN:
	- go to build/$os-Release or build/$os-Debug
	- and 'make' or 'cmake --build .'
EOF
