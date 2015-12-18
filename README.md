# CGoGN_2

CGoGN is a geometric modeling C++ library that provides an efficient implementation of combinatorial maps.

## Building and installing CGoGN
* run cmake <cgogn_path> from the build directory of your choice (this build directory must not be cgogn_path)
* you can specify build type and install path by modifying CMAKE_BUILD_TYPE and CMAKE_INSTALL_PREFIX either by using cmake-gui or ccmake, or by specifying -DCMAKE_BUILD_TYPE="<buildtype>" -DCMAKE_INSTALL_PREFIX="<install_path>" when running cmake
* CMAKE_BUILD_TYPE default value is "Release"
* LINUX and MacOS
	* make -jN or ninja in the build directory
	* make (or ninja) install in the build directory if you want to install
* Windows
	* VS 2013 or better required
	* Installation : open INSTALL solution in VS and build it


## Contribution HowTo

* Fork this GitHub repository
* Clone your GitHub fork on your working machine
* Checkout the "develop" branch
* From here, create your own "[user]" branch to commit your work in
* Push your branch with its commits in your GitHub fork
* Create a Pull Request from your "[user]" branch to the "develop" branch of this repository

If you want to update your repository with commits from other contributors, with your "[user]" branch as current branch, you can pull the "develop" branch of this repository. This will fetch new commits and merge them in your branch.
