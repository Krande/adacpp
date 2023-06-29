@echo off

:: This is a batch file to set the environment variables for the project
:: It is not strictly necessary, but it provides you with type hints when working with the OpenCascade c++ library
:: distributed using conda-forge
::
:: mamba env update -f environment.build.yml --prune
:: mamba activate ada-cpp
::
:: Note!
:: You have to add a .env file to the root of the project where you set PREFIX=<path to your ada-cpp conda env>

set MY_PY_VER=311
:: this will read the .env file and set the environment variables
for /f "delims=" %%x in ('type .env') do set "%%x"

set LIBRARY_PREFIX=%PREFIX%/Library
set CMAKE_PREFIX_PATH=%PREFIX%;%LIBRARY_PREFIX%/include;%LIBRARY_PREFIX%/lib;%LIBRARY_PREFIX%/bin

set OpenCASCADE_DIR=%LIBRARY_PREFIX%/lib/cmake/opencascade
set OpenCASCADE_INCLUDE_DIR=%LIBRARY_PREFIX%/include/opencascade

set PYTHON_EXECUTABLE=%PREFIX%/python.exe
set PYTHON_LIBRARY=%PREFIX%/libs/python%MY_PY_VER%.lib