#!/bin/bash

mkdir -p tests/build
cd tests/build
rm CMakeCache.txt
cmake -GNinja  ..
ninja