@echo off
if not exist build (
    mkdir build
    cd build
    cmake -DCMAKE_C_COMPILER=armclang -G "Unix Makefiles" ..
    make
    cd ..    
) else (
    cd build
    make
    cd ..
)