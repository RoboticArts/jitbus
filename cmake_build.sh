#!/bin/bash

mkdir -p build
cd build
cmake .. -DBUILD_WITH_AMENT=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
make -j$(nproc)
