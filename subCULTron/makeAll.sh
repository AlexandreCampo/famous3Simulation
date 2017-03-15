#!/usr/bin/env bash

for i in a*
do
    echo === Compiling $i ===
    cd $i
    premake4 gmake
    make clean
    make
    cd ..
done
