#!/bin/bash
# Build unit tests only (not turtle simulation)

mkdir -p tests/build
cd tests/build
rm -f CMakeCache.txt
cmake -GNinja ..
ninja TrackerTests