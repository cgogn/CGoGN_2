
@echo off

setlocal EnableDelayedExpansion

echo.
echo -------------- Checking for Cmake --------------
echo.

cmake --version
if  %errorlevel% ==0 (
	echo CMake found
) else (
	echo Error: CMake not found, please install it (see http://www.cmake.org)
	exit /B 1
)

