@echo on



@REM This is using scikit-build-core to compile cmake AND move python source files
@REM Switch backslashes to forward slashes in LIBRARY_PREFIX

set LIBRARY_PREFIX=%LIBRARY_PREFIX:\=/%

python -m pip install . --no-build-isolation -v ^
    --config-settings=cmake.args=-DCMAKE_INSTALL_PREFIX="%LIBRARY_PREFIX%" ^
    --config-settings=cmake.args=-DBUILD_PYTHON=ON ^
    --config-settings=cmake.args=-DBUILD_STP2GLB=ON ^
    --config-settings=cmake.args=-DSTP2GLB_BIN_DIR="%LIBRARY_PREFIX%/bin" ^
    --config-settings=cmake.args=-DBUILD_WASM=OFF

if errorlevel 1 exit 1

@REM cmake -G Ninja -B build -S . ^
@REM     -D CMAKE_INSTALL_PREFIX:FILEPATH="%LIBRARY_PREFIX%" ^
@REM     -D CMAKE_BUILD_TYPE:STRING=Release ^
@REM     -D CMAKE_PREFIX_PATH:FILEPATH="%LIBRARY_PREFIX%" ^
@REM     -D BUILD_PYTHON=ON ^
@REM     -D BUILD_STP2GLB=ON ^
@REM     -D CONDA_BUILD=ON
@REM
@REM if errorlevel 1 exit 1
@REM
@REM ninja -C build install
@REM
@REM if errorlevel 1 exit 1