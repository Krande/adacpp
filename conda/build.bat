@echo on

@REM Switch backslashes to forward slashes in LIBRARY_PREFIX
set LIBRARY_PREFIX=%LIBRARY_PREFIX:\=/%

@REM Switch backslashes to forward slashes in SP_DIR
set PYTHON_SITE_PACKAGES=%SP_DIR:\=/%

@REM Build with CMake
cmake -B build -S . -G Ninja ^
    @
    -DCMAKE_INSTALL_PREFIX="%LIBRARY_PREFIX%" ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DBUILD_PYTHON=ON ^
    -DBUILD_STP2GLB=ON ^
    -DSTP2GLB_BIN_DIR="%LIBRARY_PREFIX%/bin" ^
    -DBUILD_WASM=OFF ^
    -DPython_EXECUTABLE="%PYTHON%" ^
    -DPYTHON_INCLUDE_DIR="%LIBRARY_INC%" ^
    -DPYTHON_LIBRARY="%PYTHON_LIB%" ^
    -DPYTHON_SITE_PACKAGES="%PYTHON_SITE_PACKAGES%"

if errorlevel 1 exit 1

cmake --build build

if errorlevel 1 exit 1

cmake --install build

if errorlevel 1 exit 1

@REM Install Python package files
%PYTHON% -m pip install . --no-deps --no-build-isolation -vv

if errorlevel 1 exit 1
